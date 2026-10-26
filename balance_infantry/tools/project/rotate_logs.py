"""Keep the latest numbered change records; README and TEMPLATE are permanent."""
import argparse
from pathlib import Path
import re


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--keep", type=int, default=3)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if args.keep < 1:
        parser.error("--keep must be at least 1")
    log_dir = (Path(__file__).resolve().parents[2] / "log").resolve()
    records = sorted(
        (p for p in log_dir.iterdir()
         if p.is_file() and re.fullmatch(r"\d{3}-[^/\\]+\.md", p.name)),
        key=lambda p: int(p.name[:3]), reverse=True)
    for path in records[args.keep:]:
        # Never follow a link outside the explicitly named log directory.
        if path.resolve().parent != log_dir:
            raise RuntimeError(f"Refusing out-of-directory record: {path}")
        print(("Would remove " if args.dry_run else "Removed ") + path.name)
        if not args.dry_run:
            path.unlink()
    if not args.dry_run:
        index = log_dir / "README.md"
        if index.exists():
            text = index.read_text(encoding="utf-8")
            rows = []
            for path in records[:args.keep]:
                title = path.read_text(encoding="utf-8").splitlines()[0].lstrip("# ")
                rows.append(f"- [{title}]({path.name})")
            text = re.sub(
                r"<!-- RECENT_LOGS_START -->.*?<!-- RECENT_LOGS_END -->",
                "<!-- RECENT_LOGS_START -->\n" + "\n".join(rows) + "\n<!-- RECENT_LOGS_END -->",
                text, flags=re.S)
            index.write_text(text, encoding="utf-8", newline="\n")
    print("Retained: " + ", ".join(p.name for p in records[:args.keep]))


if __name__ == "__main__":
    main()
