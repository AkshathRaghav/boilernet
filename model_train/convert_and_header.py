import os
import argparse
import tensorflow as tf

def representative_data_gen(data_dir, subset_synsets, image_size, num_samples=100):
    """Yields representative samples for INT8 quantization from the specified subset."""
    image_paths = []
    for syn in subset_synsets:
        syn_dir = os.path.join(data_dir, "train", syn)
        if os.path.isdir(syn_dir):
            for f in os.listdir(syn_dir):
                if f.lower().endswith(('.jpg', '.jpeg', '.png')):
                    image_paths.append(os.path.join(syn_dir, f))
    # Limit to num_samples
    image_paths = image_paths[:num_samples]

    def preprocess(path):
        img = tf.io.read_file(path)
        img = tf.image.decode_image(img, channels=3)
        img = tf.image.resize(img, [image_size, image_size])
        img = tf.cast(img, tf.float32) / 255.0
        return img

    for path in image_paths:
        img = preprocess(path)
        img = tf.expand_dims(img, axis=0)
        yield [img]

def quantize_model(saved_model_dir, tflite_path, data_dir, subset_synsets, image_size):
    keras_model = tf.keras.models.load_model(saved_model_dir)  # works with .keras or .h5
    converter = tf.lite.TFLiteConverter.from_keras_model(keras_model)

    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    converter.inference_input_type = tf.int8
    converter.inference_output_type = tf.int8
    converter.representative_dataset = lambda: representative_data_gen(data_dir, subset_synsets, image_size)
    tflite_model = converter.convert()
    os.makedirs(os.path.dirname(tflite_path), exist_ok=True)
    with open(tflite_path, 'wb') as f:
        f.write(tflite_model)
    print(f"Quantized TFLite model saved to {tflite_path}")
    return tflite_model

def generate_cc_header(tflite_model_bytes, array_name, header_path):
    """Convert tflite byte model into a C array in a .cc file."""
    os.makedirs(os.path.dirname(header_path), exist_ok=True)
    with open(header_path, 'w') as f:
        f.write('// Generated header for model\n')
        f.write('#include <cstdint>\n\n')
        f.write(f'const unsigned char {array_name}[] = {{\n')
        for i, b in enumerate(tflite_model_bytes):
            if i % 12 == 0:
                f.write('    ')
            f.write(f'0x{b:02x}, ')
            if i % 12 == 11:
                f.write('\n')
        f.write('\n};\n')
        f.write(f'const unsigned int {array_name}_len = {len(tflite_model_bytes)};\n')
    print(f"C header generated at {header_path}")

# ─── Subset Definition ──────────────────────────────────────────────────────────
object_subsets = {
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

def main():
    parser = argparse.ArgumentParser(description="Convert TF SavedModel to int8 TFLite + generate C header for multiple subsets.")
    parser.add_argument(
        "--subset",
        required=True,
        nargs="+",  
        choices=list(object_subsets.keys()),
        help='Which subsets to use for representative data'
    )
    parser.add_argument('--saved_model_template', required=True, help='File path pattern of the TF Keras model.')
    parser.add_argument('--data_dir',        required=True, 
                        help='Path to ILSVRC2012 with train/<synset>/ and val/<synset>/')

    parser.add_argument('--tflite_path',     required=True, help='Output folder for the .tflite file.')
    parser.add_argument('--header_path',     required=True, help='Output folder for the generated C header (.cc).')
    parser.add_argument('--array_name',      default='model_tflite', help='C array name for the model data.')
    parser.add_argument('--image_size',      type=int, default=32, help='Image input size for representative data.')
    args = parser.parse_args()

    for subset_ in args.subset:
        print(f"Converting first subset: {subset_}")
        synsets = object_subsets[subset_]
        header_path = os.path.join(args.header_path, f"{subset_}_model_data.cc")
        tflite_path = os.path.join(args.tflite_path, f"{subset_}_int8.tflite")
        saved_model = args.saved_model_template.format(subset_)

        tflite_bytes = quantize_model(
            saved_model, tflite_path,
            args.data_dir, synsets, args.image_size)

        generate_cc_header(tflite_bytes, args.array_name, header_path)

if __name__ == "__main__":
    main()


# python convert_and_header.py --saved_model_template "/home/araviki/workbench/boilernet/model_train/models/{}_mobilenet_v2_best.keras"  --data_dir "/home/araviki/workbench/boilernet/model_train/imagenet" --subset dogs cats kitchen fruits stationary --tflite_path "/home/araviki/workbench/boilernet/model_train/quantized_models" --header_path  "/home/araviki/workbench/boilernet/model_train/quantized_models" --image_size 224
