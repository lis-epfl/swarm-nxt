#!/usr/bin/env python3
"""
Recover an mcap file that's missing its trailing summary/footer.

The oak_compressed_recorder writes a valid stream of mcap records to disk,
but on `systemctl stop ros_packages` the recorder process is sometimes
killed by systemd before its writer.close() finishes writing the summary
section + footer.  The data records are intact; the file just lacks the
end-of-file index that mcap readers seek to first.

This script reads the broken file sequentially (StreamReader doesn't need
the footer), copies every Schema/Channel/Message into a fresh writer, and
finishes with a proper Footer.  The output is bit-equivalent in payload
content; only the metadata wrapper is regenerated.

Usage:
    recover_mcap.py <broken.mcap> <fixed.mcap>
"""
import os
import sys

from mcap.exceptions import EndOfFile, McapError
from mcap.records import Channel, Message, Schema
from mcap.stream_reader import StreamReader
from mcap.writer import CompressionType, Writer


def main():
    if len(sys.argv) != 3:
        print("usage: recover_mcap.py <broken.mcap> <fixed.mcap>", file=sys.stderr)
        sys.exit(2)

    src_path, dst_path = sys.argv[1:3]
    if not os.path.isfile(src_path):
        print(f"error: source {src_path!r} does not exist", file=sys.stderr)
        sys.exit(1)

    print(f"reading {src_path} ({os.path.getsize(src_path) / 1e9:.2f} GB)")
    print(f"writing {dst_path}")

    schema_id_map = {}
    channel_id_map = {}
    n_msgs = 0

    with open(src_path, "rb") as src, open(dst_path, "wb") as dst:
        w = Writer(dst, compression=CompressionType.NONE)
        w.start(profile="ros2", library="oak_compressed_recorder.recover")

        rdr = StreamReader(src, skip_magic=False)
        try:
            for rec in rdr.records:
                if isinstance(rec, Schema):
                    schema_id_map[rec.id] = w.register_schema(
                        name=rec.name, encoding=rec.encoding, data=bytes(rec.data)
                    )
                elif isinstance(rec, Channel):
                    new_sid = schema_id_map.get(rec.schema_id, 0)
                    channel_id_map[rec.id] = w.register_channel(
                        topic=rec.topic,
                        message_encoding=rec.message_encoding,
                        schema_id=new_sid,
                    )
                elif isinstance(rec, Message):
                    new_cid = channel_id_map.get(rec.channel_id)
                    if new_cid is None:
                        continue
                    w.add_message(
                        channel_id=new_cid,
                        log_time=rec.log_time,
                        publish_time=rec.publish_time,
                        sequence=rec.sequence,
                        data=bytes(rec.data),
                    )
                    n_msgs += 1
                    if n_msgs % 1000 == 0:
                        print(f"  {n_msgs} msgs", end="\r", flush=True)
        except (EndOfFile, McapError) as e:
            # Expected: original file lacked footer.
            print(f"\nreader stopped at end-of-stream ({type(e).__name__})")

        w.finish()

    print(f"wrote {n_msgs} messages to {dst_path} ({os.path.getsize(dst_path) / 1e9:.2f} GB)")


if __name__ == "__main__":
    main()
