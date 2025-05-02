#!/usr/bin/env python3
"""
inference.py

Run INT8 TFLite inference with proper quantization handling,
using MobileNetV2-style preprocessing and printing raw vs. dequantized outputs.
"""

import os
import argparse
import numpy as np
import tensorflow as tf
from tensorflow.keras.applications.mobilenet_v2 import preprocess_input

# ─── subset → synset map (same as your convert.py) ────────────────────────────────
OBJECT_SUBSETS = {
    "dogs": [
        "n02106662",  # German shepherd
        "n02099712",  # Labrador retriever
        "n02094433",  # Pug
        "n02085620",  # Chihuahua
        "n02113799",  # Poodle
    ],
    "cats": [
        "n02124075",  # Egyptian cat
        "n02125311",  # Persian cat
        "n02129165",  # Lion
        "n02132136",  # Bear
        "n02123394",  # Baboon
    ],
    "fruits": [
        "n07753592",  # banana 
        "n07745940",  # strawberry 
        "n07753275",  # pineapple
        "n07747607",  # orange 
        "n07760859",  # custard apple 
    ],
    "kitchen": [
        "n03400231",  # frypan
        "n03761084",  # microwave 
        "n03775546",  # mixing bowl
        "n04398044",  # teapot 
        "n07579787",  # plate
    ],
    "stationary": [
        "n03179701",  # desk
        "n03291819",  # envelope
        "n03018349",  # cabinet 
        "n03832673",  # notebook
        "n03180011",  # computer/monitor
    ]
}

def load_and_preprocess(path, image_size):
    """Load an image file and run MobileNetV2 preprocess_input (–1 to +1)."""
    img = tf.io.read_file(path)
    img = tf.image.decode_image(img, channels=3)
    img = tf.image.resize(img, [image_size, image_size])
    img = tf.cast(img, tf.float32)
    img = preprocess_input(img)  # now in [–1,1]
    return img.numpy()

def collect_image_paths(data_dir, subset, max_images=None):
    """Gather up to max_images from data_dir/val/<synset>/ for the given subset."""
    synsets = OBJECT_SUBSETS[subset]
    all_paths = []
    for syn in synsets:
        folder = os.path.join(data_dir, "val", syn)
        if not os.path.isdir(folder):
            continue
        for fn in os.listdir(folder):
            if fn.lower().endswith((".jpg", ".jpeg", ".png")):
                all_paths.append(os.path.join(folder, fn))
    all_paths.sort()
    return all_paths[:max_images] if max_images else all_paths

def run_inference(
    tflite_model_path,
    image_paths,
    image_size,
    top_k=3
):
    # Load interpreter
    interpreter = tf.lite.Interpreter(model_path=tflite_model_path)
    interpreter.allocate_tensors()

    # Get quantization details
    input_det  = interpreter.get_input_details()[0]
    output_det = interpreter.get_output_details()[0]
    scale_in, zp_in   = input_det["quantization"]
    scale_out, zp_out = output_det["quantization"]

    print(f"\n-- Quant Params --\n"
          f" input  dtype={input_det['dtype']}, scale={scale_in}, zero_pt={zp_in}\n"
          f" output dtype={output_det['dtype']}, scale={scale_out}, zero_pt={zp_out}\n")

    for img_path in image_paths:
        # 1) load + preprocess → float32 [–1,1]
        img = load_and_preprocess(img_path, image_size)[None, ...]  # add batch dim

        # 2) float → int8
        int8_input = np.round(img / scale_in + zp_in).astype(np.int8)

        # 3) run
        interpreter.set_tensor(input_det["index"], int8_input)
        interpreter.invoke()

        # 4) pull raw int8 logits
        raw_out = interpreter.get_tensor(output_det["index"])[0]  # shape [num_classes]

        # 5) dequantize → float logits
        logits = (raw_out.astype(np.float32) - zp_out) * scale_out

        # 6) softmax
        probs = tf.nn.softmax(logits).numpy()

        # 7) display
        top_idxs = np.argsort(probs)[::-1][:top_k]
        print(f"\nImage: {os.path.basename(img_path)}")
        print(" Raw quantized output:", raw_out.tolist())
        print(" Dequantized logits  :", np.round(logits, 4).tolist())
        print(" Softmax probabilities:", np.round(probs, 4).tolist())
        print(f" Top {top_k} predictions:")
        for i in top_idxs:
            print(f"  - Class {i:3d} → {probs[i]:.4f}")
        print("-" * 50)

def main():
    parser = argparse.ArgumentParser(
        description="Run INT8 TFLite inference with correct quantization handling"
    )
    parser.add_argument(
        "--model", "-m", required=True,
        help="Path to your int8 .tflite file"
    )
    parser.add_argument(
        "--data_dir", "-d", required=True,
        help="Path to ImageNet subset root, e.g. ILSVRC2012"
    )
    parser.add_argument(
        "--subset", "-s", required=True, choices=OBJECT_SUBSETS.keys(),
        help="Which subset (dogs, cats, etc.) to pull validation images from"
    )
    parser.add_argument(
        "--image_size", type=int, default=32,
        help="Image size used during quantization (default: 32)"
    )
    parser.add_argument(
        "--max_images", type=int, default=5,
        help="How many images to run (default: 5)"
    )
    parser.add_argument(
        "--top_k", type=int, default=3,
        help="How many top predictions to show (default: 3)"
    )
    args = parser.parse_args()

    imgs = collect_image_paths(args.data_dir, args.subset, args.max_images)
    if not imgs:
        raise ValueError(f"No images found for subset '{args.subset}' in {args.data_dir}/val/...")
    run_inference(
        tflite_model_path=args.model,
        image_paths=imgs,
        image_size=args.image_size,
        top_k=args.top_k
    )

if __name__ == "__main__":
    main()

# python infer_quant.py  --data_dir "/home/araviki/workbench/boilernet/model_train/imagenet" --subset cats --model "/home/araviki/workbench/boilernet/model_train/quantized_models/cats_int8.tflite"  --image_size 224 --max_images 5 --top_k 5
