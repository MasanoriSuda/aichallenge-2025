import json
from pathlib import Path
import random
from datetime import datetime

import numpy as np
import torch
import torch.optim as optim
from torch.utils.data import DataLoader
from tqdm import tqdm
import hydra
from omegaconf import DictConfig, OmegaConf
from torch.utils.tensorboard import SummaryWriter

from lib.checkpoint import load_pretrained_weights
from lib.data import MultiSeqConcatDataset, assert_disjoint_sequence_ids
from lib.loss import WeightedSmoothL1Loss
from lib.model import TinyLidarNet, TinyLidarNetSmall


def select_trainable_parameters(
    model: torch.nn.Module, trainable_layers
) -> tuple[list[torch.nn.Parameter], list[str]]:
    """Freeze all but explicitly selected top-level layers.

    An empty selection preserves historical full-network fine tuning.  Keeping
    the selection in the training manifest makes a sparse corrective update
    auditable rather than an implicit optimizer side effect.
    """
    selected_layers = tuple(str(layer) for layer in trainable_layers)
    known_layers = {name for name, _ in model.named_children()}
    unknown = sorted(set(selected_layers) - known_layers)
    if unknown:
        raise ValueError(
            f"Unknown trainable layers: {unknown}; known={sorted(known_layers)}"
        )

    trainable = []
    trainable_names = []
    for name, parameter in model.named_parameters():
        layer = name.split(".", 1)[0]
        parameter.requires_grad = not selected_layers or layer in selected_layers
        if parameter.requires_grad:
            trainable.append(parameter)
            trainable_names.append(name)
    if not trainable:
        raise ValueError("trainable layer selection produced no parameters")
    return trainable, trainable_names



def clean_numerical_tensor(x: torch.Tensor) -> torch.Tensor:
    """NaN, infを安全に除去"""
    if torch.isnan(x).any() or torch.isinf(x).any():
        x = torch.nan_to_num(x, nan=0.0, posinf=0.0, neginf=0.0)
    return x


def seed_everything(seed: int) -> torch.Generator:
    """Make model initialization and DataLoader shuffling reproducible."""
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    generator = torch.Generator()
    generator.manual_seed(seed)
    return generator


def write_training_manifest(
    path: Path,
    cfg: DictConfig,
    train_dataset: MultiSeqConcatDataset,
    val_dataset: MultiSeqConcatDataset,
    pretrained: dict,
) -> None:
    manifest = {
        "schema_version": 1,
        "config": OmegaConf.to_container(cfg, resolve=True),
        "train_sequence_ids": train_dataset.sequence_ids,
        "val_sequence_ids": val_dataset.sequence_ids,
        "train_samples": len(train_dataset),
        "val_samples": len(val_dataset),
        "pretrained": pretrained,
    }
    path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


@hydra.main(config_path="./config", config_name="train", version_base="1.2")
def main(cfg: DictConfig):
    print("------ Configuration ------")
    print(OmegaConf.to_yaml(cfg))
    print("---------------------------")

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")
    data_generator = seed_everything(int(cfg.train.seed))

    # === Dataset ===
    dataset_contract = {
        "max_range": cfg.data.max_range,
        "expected_input_dim": cfg.model.input_dim,
        "allowed_label_sources": list(cfg.data.allowed_label_sources),
        "require_metadata": cfg.data.require_metadata,
        "max_sync_delta_sec": cfg.data.max_sync_delta_sec,
    }
    train_dataset = MultiSeqConcatDataset(
        cfg.data.train_dir,
        expected_split="train",
        **dataset_contract,
    )
    val_dataset = MultiSeqConcatDataset(
        cfg.data.val_dir,
        expected_split="val",
        **dataset_contract,
    )
    assert_disjoint_sequence_ids(
        train_dataset.sequence_ids, val_dataset.sequence_ids
    )

    train_loader = DataLoader(
        train_dataset,
        batch_size=cfg.train.batch_size,
        shuffle=True,
        num_workers=cfg.train.num_workers,
        pin_memory=device.type == "cuda",
        drop_last=True,
        generator=data_generator,
    )

    val_loader = DataLoader(
        val_dataset,
        batch_size=cfg.train.batch_size,
        shuffle=False,
        num_workers=cfg.train.num_workers,
        pin_memory=device.type == "cuda",
        drop_last=False,
    )

    # === Model ===
    if cfg.model.name == "TinyLidarNetSmall":
        model = TinyLidarNetSmall(
            input_dim=cfg.model.input_dim,
            output_dim=cfg.model.output_dim
        ).to(device)
    else:
        model = TinyLidarNet(
            input_dim=cfg.model.input_dim,
            output_dim=cfg.model.output_dim
        ).to(device)

    pretrained = {}
    if cfg.train.pretrained_path:
        pretrained = load_pretrained_weights(
            model, Path(cfg.train.pretrained_path)
        )
        print(
            "[INFO] Loaded pretrained model "
            f"format={pretrained['format']} sha256={pretrained['sha256']}"
        )

    # === Loss & Optimizer ===
    criterion = WeightedSmoothL1Loss(
        steer_weight=cfg.train.loss.steer_weight,
        accel_weight=cfg.train.loss.accel_weight
    )
    trainable_parameters, trainable_names = select_trainable_parameters(
        model, cfg.train.get("trainable_layers", [])
    )
    print(
        "[INFO] Trainable parameters: "
        f"{sum(parameter.numel() for parameter in trainable_parameters)} "
        f"in {trainable_names}"
    )
    optimizer = optim.Adam(trainable_parameters, lr=cfg.train.lr)

    # === Logging & Save dirs ===
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_dir = Path(cfg.train.save_dir).expanduser().resolve()
    log_dir = Path(cfg.train.log_dir).expanduser().resolve()
    run_save_dir = save_dir / timestamp
    run_save_dir.mkdir(parents=True, exist_ok=False)
    log_dir.mkdir(parents=True, exist_ok=True)
    write_training_manifest(
        run_save_dir / "training-manifest.json",
        cfg,
        train_dataset,
        val_dataset,
        pretrained,
    )

    with SummaryWriter(log_dir / timestamp) as writer:
        best_val_loss = float("inf")
        patience_counter = 0
        max_patience = cfg.train.get("early_stop_patience", 10)

        best_path = run_save_dir / "best_model.pth"
        last_path = run_save_dir / "last_model.pth"

        # === Training Loop ===
        for epoch in range(cfg.train.epochs):
            model.train()
            train_loss = 0.0

            for scans, targets in tqdm(
                train_loader,
                desc=f"[Train] Epoch {epoch + 1}/{cfg.train.epochs}",
            ):
                scans = scans.unsqueeze(1).to(device)
                targets = targets.to(device)

                scans = clean_numerical_tensor(scans)
                targets = clean_numerical_tensor(targets)

                outputs = model(scans)
                loss = criterion(outputs, targets)

                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
                train_loss += loss.item()

            avg_train_loss = train_loss / len(train_loader)
            avg_val_loss = validate(model, val_loader, device, criterion)

            print(
                f"Epoch {epoch + 1:03d}: "
                f"Train={avg_train_loss:.4f} | Val={avg_val_loss:.4f}"
            )
            writer.add_scalar("Loss/train", avg_train_loss, epoch + 1)
            writer.add_scalar("Loss/val", avg_val_loss, epoch + 1)

            if avg_val_loss < best_val_loss:
                best_val_loss = avg_val_loss
                torch.save(model.state_dict(), best_path)
                print(f"[SAVE] Best model updated: {best_path} (val_loss={best_val_loss:.4f})")
                patience_counter = 0
            else:
                patience_counter += 1

            torch.save(model.state_dict(), last_path)
            if patience_counter >= max_patience:
                print(f"[EarlyStop] No improvement for {max_patience} epochs.")
                break
    
    print("Training finished.")


def validate(model, loader, device, criterion):
    model.eval()
    total_loss = 0.0
    with torch.no_grad():
        for scans, targets in tqdm(loader, desc="[Val]", leave=False):
            scans = scans.unsqueeze(1).to(device)
            targets = targets.to(device)
            scans = clean_numerical_tensor(scans)
            targets = clean_numerical_tensor(targets)
            outputs = model(scans)
            loss = criterion(outputs, targets)
            total_loss += loss.item()
    return total_loss / len(loader)


if __name__ == "__main__":
    main()
