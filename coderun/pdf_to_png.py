"""
Convert all PDF files in current directory and subdirectories to PNG images.
Each PDF gets its own folder containing one PNG per page.

Dependencies:
    pip install pymupdf
"""

import sys
from pathlib import Path

try:
    import fitz  # pymupdf
except ImportError:
    print("pymupdf not found. Installing...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pymupdf"])
    import fitz


def pdf_to_pngs(pdf_path: Path, dpi: int = 150) -> Path:
    """Convert a PDF to PNG images, one per page, in a dedicated folder."""
    # Create output folder next to the PDF (same name without extension)
    output_dir = pdf_path.parent / pdf_path.stem
    output_dir.mkdir(exist_ok=True)

    doc = fitz.open(pdf_path)
    zoom = dpi / 72  # 72 is the default PDF DPI
    matrix = fitz.Matrix(zoom, zoom)

    for page_num in range(len(doc)):
        page = doc[page_num]
        pix = page.get_pixmap(matrix=matrix)
        png_path = output_dir / f"page_{page_num + 1:03d}.png"
        pix.save(png_path)

    doc.close()
    return output_dir


def main():
    input_dir = Path(__file__).parent

    if not input_dir.exists():
        print(f"Directory not found: {input_dir}")
        sys.exit(1)

    # Recursively find all PDF files
    pdfs = list(input_dir.rglob("*.pdf"))
    if not pdfs:
        print(f"No PDF files found in {input_dir} or subdirectories")
        sys.exit(0)

    print(f"Found {len(pdfs)} PDF(s) in {input_dir} and subdirectories\n")

    for pdf in pdfs:
        rel_path = pdf.relative_to(input_dir)
        print(f"Converting: {rel_path} ...", end=" ", flush=True)
        try:
            out_dir = pdf_to_pngs(pdf)
            page_count = len(list(out_dir.glob("*.png")))
            print(f"-> {out_dir.name}/ ({page_count} pages)")
        except Exception as e:
            print(f"FAILED ({e})")

    print("\nDone.")


if __name__ == "__main__":
    main()
