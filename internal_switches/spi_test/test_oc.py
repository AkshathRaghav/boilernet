#!/usr/bin/env python3
import sys
import os
import lz4.frame

def main():
    if len(sys.argv) < 2:
        print("Usage: python lz4_chunk_compression.py <filename>")
        return

    filename = sys.argv[1]
    if not os.path.exists(filename):
        print("File not found:", filename)
        return

    with open(filename, 'rb') as f:
        data = f.read()

    total_size = len(data)
    print("Original file size: {} bytes".format(total_size))

    # Set number of chunks
    num_chunks = 8
    chunk_size = total_size // num_chunks
    total_compressed = 0

    print("\nCompressing in {} chunks:".format(num_chunks))
    for i in range(num_chunks):
        start = i * chunk_size
        # Make sure the last chunk takes any remaining bytes.
        if i == num_chunks - 1:
            chunk = data[start:]
        else:
            chunk = data[start:start+chunk_size]
        compressed = lz4.frame.compress(chunk)
        comp_size = len(compressed)
        total_compressed += comp_size
        print("  Chunk {:d}: Original = {:d} bytes, Compressed = {:d} bytes".format(i+1, len(chunk), comp_size))

    print("\nTotal compressed size (8 chunks): {} bytes".format(total_compressed))

    # Also compress the entire file in one block for comparison.
    full_compressed = lz4.frame.compress(data)
    print("Compressed size (entire file): {} bytes".format(len(full_compressed)))

if __name__ == '__main__':
    main()
