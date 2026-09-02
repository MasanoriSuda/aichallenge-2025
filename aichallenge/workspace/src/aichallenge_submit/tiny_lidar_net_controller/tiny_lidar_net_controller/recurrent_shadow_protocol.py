"""Private framed protocol shared by the recurrent parent and worker."""

import pickle
import struct


_HEADER = struct.Struct("!Q")


def _read_exact(stream, size: int) -> bytes:
    chunks = []
    remaining = int(size)
    while remaining:
        chunk = stream.read(remaining)
        if not chunk:
            raise EOFError("recurrent worker pipe closed")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def send_message(stream, value) -> None:
    payload = pickle.dumps(value, protocol=pickle.HIGHEST_PROTOCOL)
    stream.write(_HEADER.pack(len(payload)))
    stream.write(payload)
    stream.flush()


def receive_message(stream):
    payload_size = _HEADER.unpack(_read_exact(stream, _HEADER.size))[0]
    if payload_size > 64 * 1024 * 1024:
        raise ValueError("recurrent worker frame exceeds 64 MiB")
    return pickle.loads(_read_exact(stream, payload_size))
