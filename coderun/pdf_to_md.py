"""
Convert all PDF files in current directory and subdirectories to Markdown files.
Output .md files are saved in the same directory alongside the PDFs.

Dependencies:
    pip install pymupdf4llm
"""

import sys
from pathlib import Path

try:
    import pymupdf4llm
except ImportError:
    print("pymupdf4llm not found. Installing...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pymupdf4llm"])
    import pymupdf4llm


def pdf_to_md(pdf_path: Path, output_dir: Path) -> Path:
    """Convert a single PDF to Markdown and write it to output_dir."""
    md_text = pymupdf4llm.to_markdown(str(pdf_path), use_ocr=False)
    md_path = output_dir / (pdf_path.stem + ".md")
    md_path.write_text(md_text, encoding="utf-8")
    return md_path


def main():
    input_dir = Path(__file__).parent

    if not input_dir.exists():
        print(f"Directory not found: {input_dir}")
        sys.exit(1)

    # Recursively find all PDF files in subdirectories
    pdfs = list(input_dir.rglob("*.pdf"))
    if not pdfs:
        print(f"No PDF files found in {input_dir} or subdirectories")
        sys.exit(0)

    print(f"Found {len(pdfs)} PDF(s) in {input_dir} and subdirectories\n")

    for pdf in pdfs:
        # Get relative path for better display
        rel_path = pdf.relative_to(input_dir)
        print(f"Converting: {rel_path} ...", end=" ", flush=True)
        try:
            # Save MD file in same directory as PDF
            out = pdf_to_md(pdf, pdf.parent)
            print(f"-> {out.name}")
        except Exception as e:
            print(f"FAILED ({e})")

    print("\nDone.")


if __name__ == "__main__":
    main()
