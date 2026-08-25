from pathlib import Path
import re


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]


def read(relative_path: str) -> str:
    return (REPOSITORY_ROOT / relative_path).read_text(encoding="utf-8")


def test_make_and_compose_propagate_scenario_vehicle_count() -> None:
    makefile = read("Makefile")
    compose = read("docker-compose.yml")

    assert re.search(r"dev:\s+AIC_VEHICLE_COUNT\s*:=\s*1", makefile)
    assert "AIC_VEHICLE_COUNT=$$N" in makefile
    assert "AIC_VEHICLE_COUNT=${AIC_VEHICLE_COUNT:-1}" in compose
