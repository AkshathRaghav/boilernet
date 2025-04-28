#!/usr/bin/env python3
import os
import argparse
import tensorflow as tf
from tensorflow.keras import layers, models, callbacks, optimizers

# ─── YOUR SUBSET DEFINITIONS ──────────────────────────────────────────────────
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
    ]
}

# ─── Backbone Registry ────────────────────────────────────────────────────────
BACKBONES = {
    "mobilenet_v2": (
        tf.keras.applications.MobileNetV2,
        tf.keras.applications.mobilenet_v2.preprocess_input,
        {"alpha": 0.35}
    ),
    "mobilenet_v3_small": (
        tf.keras.applications.MobileNetV3Small,
        tf.keras.applications.mobilenet_v3.preprocess_input,
        {"alpha": 0.35}
    ),
    "efficientnet_b0": (
        tf.keras.applications.EfficientNetB0,
        tf.keras.applications.efficientnet.preprocess_input,
        {}
    )
}

# ─── Data Loading for a Single Subset ─────────────────────────────────────────
def make_subset_dataset(data_root, subset_name, split, image_size, batch_size,
                        preprocess_fn, training):
    synsets = object_subsets[subset_name]
    file_paths, labels = [], []

    for idx, syn in enumerate(synsets):
        dir_path = os.path.join(data_root, split, syn)
        for fname in os.listdir(dir_path):
            if fname.lower().endswith((".jpg", ".jpeg", ".jpeg")):
                file_paths.append(os.path.join(dir_path, fname))
                labels.append(idx)

    ds = tf.data.Dataset.from_tensor_slices((file_paths, labels))
    if training:
        ds = ds.shuffle(len(file_paths), seed=123)
    def load_and_pre(path, label):
        img = tf.io.read_file(path)
        img = tf.image.decode_jpeg(img, channels=3)
        img = tf.image.resize(img, [image_size, image_size])
        img = tf.cast(img, tf.float32)
        # backbone‑specific preprocessing
        img = preprocess_fn(img)
        return img, label

    ds = ds.map(load_and_pre, num_parallel_calls=tf.data.AUTOTUNE)

    if training:
        aug = tf.keras.Sequential([
            layers.RandomFlip("horizontal"),
            layers.RandomRotation(0.1),
            layers.RandomZoom(0.1),
            layers.RandomContrast(0.1),
        ], name="data_augmentation")
        ds = ds.map(lambda x, y: (aug(x, training=True), y),
                    num_parallel_calls=tf.data.AUTOTUNE)

    ds = ds.batch(batch_size).prefetch(tf.data.AUTOTUNE)
    return ds

# ─── Model Builder ───────────────────────────────────────────────────────────
def build_model(backbone_name, input_shape, num_classes):
    Backbone, preprocess_fn, kwargs = BACKBONES[backbone_name]
    # instantiate backbone
    base = Backbone(
        input_shape=input_shape,
        include_top=False,
        weights="imagenet",
        pooling="avg",
        **kwargs
    )
    base.trainable = False

    inputs = layers.Input(shape=input_shape)
    x = base(inputs, training=False)
    x = layers.Dropout(0.3)(x)
    outputs = layers.Dense(num_classes, activation="softmax")(x)
    model = models.Model(inputs, outputs, name=f"{backbone_name}_subset")
    return model, base, preprocess_fn

# ─── Training Entry Point ────────────────────────────────────────────────────
def main():
    p = argparse.ArgumentParser()
    p.add_argument("--data_dir",    required=True,
                   help="Path to imagenet folder with train/ and val/")
    p.add_argument("--subset",      required=True,
                   choices=list(object_subsets.keys()),
                   help="Which subset to train (e.g. dogs or cats)")
    p.add_argument("--output_dir",  required=True,
                   help="Where to save checkpoints & final model")

    p.add_argument("--backbone",    required=True,
                   choices=list(BACKBONES.keys()),
                   help="Which pretrained backbone to use")
    p.add_argument("--image_size",  type=int, default=224,
                   help="Resize both dimensions to this for training")
    p.add_argument("--batch_size",  type=int, default=64)
    p.add_argument("--head_epochs", type=int, default=5,
                   help="Epochs to train only the classification head")
    p.add_argument("--total_epochs",type=int, default=30,
                   help="Total epochs including head_epochs")
    args = p.parse_args()

    model, backbone, preprocess_fn = build_model(
        args.backbone,
        input_shape=(args.image_size, args.image_size, 3),
        num_classes=len(object_subsets[args.subset])
    )
    train_ds = make_subset_dataset(
        args.data_dir, args.subset, "train", args.image_size,
        args.batch_size, preprocess_fn, training=True)
    val_ds   = make_subset_dataset(
        args.data_dir, args.subset, "val",   args.image_size,
        args.batch_size, preprocess_fn, training=False)

    optimizer = optimizers.Adam(learning_rate=1e-3)
    model.compile(
        optimizer=optimizer,
        loss="sparse_categorical_crossentropy",
        metrics=["accuracy"]
    )
    model.summary()

    os.makedirs(args.output_dir, exist_ok=True)
    ckpt = os.path.join(args.output_dir, f"{args.subset}_{args.backbone}_best.keras")
    cbs = [
        callbacks.ModelCheckpoint(ckpt, monitor="val_accuracy",
                                  save_best_only=True, verbose=1),
        callbacks.ReduceLROnPlateau(monitor="val_loss",
                                    factor=0.5, patience=3, verbose=1),
    ]

    # ─── Phase 1: head only ─────────────────────────────────────────────────────
    print(f"\n>> Phase 1: training head for {args.head_epochs} epochs on subset [{args.subset}]")
    model.fit(train_ds, validation_data=val_ds,
              epochs=args.head_epochs, callbacks=cbs, verbose=2)

    # ─── Phase 2: fine‑tune backbone ───────────────────────────────────────────
    print("\n>> Phase 2: fine‑tuning backbone")
    backbone.trainable = True
    for layer in backbone.layers[:-20]:
        layer.trainable = False

    model.compile(
        optimizer=optimizers.Adam(learning_rate=1e-4),
        loss="sparse_categorical_crossentropy",
        metrics=["accuracy"]
    )
    model.fit(train_ds, validation_data=val_ds,
              initial_epoch=args.head_epochs,
              epochs=args.total_epochs,
              callbacks=cbs, verbose=2)

    out_path = os.path.join(args.output_dir,
                            f"{args.subset}_{args.backbone}_final.keras")
    model.save(out_path)
    print("✅ Training complete. Model saved to:", out_path)

if __name__ == "__main__":
    main()


# python train_full.py --subset cats --data_dir "/home/araviki/workbench/boilernet/model_train/imagenet" --output_dir "/home/araviki/workbench/boilernet/model_train/models" --backbone "mobilenet_v2" --image_size 224 --batch_size 32 --head_epochs 50 --total_epochs 100
