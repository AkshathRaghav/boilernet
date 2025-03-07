import sys

def compare_files(file1, file2):
    try:
        with open(file1, 'r') as f1, open(file2, 'r') as f2:
            line_num = 0
            for line1, line2 in zip(f1, f2):
                line_num += 1
                if line1.strip() != line2.strip():  # Ignore whitespace differences
                    print(f"❌ Files are different at line {line_num}:\n"
                          f"File 1: {line1.strip()}\n"
                          f"File 2: {line2.strip()}")
                    return False

            # Check if one file has extra lines
            extra_lines1 = f1.readlines()
            extra_lines2 = f2.readlines()

            if extra_lines1 or extra_lines2:
                print(f"❌ Files have different lengths.")
                return False

        print("✅ Files are identical!")
        return True

    except FileNotFoundError as e:
        print(f"Error: {e}")
        return False

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python compare_text_files.py <file1.txt> <file2.txt>")
    else:
        file1, file2 = sys.argv[1], sys.argv[2]
        compare_files(file1, file2)
