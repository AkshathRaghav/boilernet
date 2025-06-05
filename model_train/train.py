#!/usr/bin/env python3
import os
import argparse
import pandas as pd
import matplotlib.pyplot as plt
import logging

import tensorflow as tf
from tensorflow.keras import layers, models, callbacks, optimizers

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

def build_model(backbone_name, input_shape, num_classes):
    Backbone, preprocess_fn, kwargs = BACKBONES[backbone_name]
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

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--data_dir",    required=True,
                   help="Path to imagenet folder with train/ and val/")
    p.add_argument(
        "--subset",
        required=True,
        nargs="+",  
        choices=list(object_subsets.keys()),
        help="One or more subsets to train (e.g. fruits vehicles tools)"
    )
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


    os.makedirs(args.output_dir, exist_ok=True)

    for subset_ in args.subset:
        log_path = os.path.join(args.output_dir, f"{subset_}_{args.backbone}.log")
        logger = logging.getLogger(subset_ + args.backbone)
        logger.setLevel(logging.INFO)
        if not logger.handlers:
            fh = logging.FileHandler(log_path)
            fh.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
            logger.addHandler(fh)

        logger.info(f"Starting training for subset={subset_}, backbone={args.backbone}")

        model, backbone, preprocess_fn = build_model(
            args.backbone,
            input_shape=(args.image_size, args.image_size, 3),
            num_classes=len(object_subsets[subset_])
        )
        train_ds = make_subset_dataset(
            args.data_dir, subset_, "train", args.image_size,
            args.batch_size, preprocess_fn, training=True)
        val_ds   = make_subset_dataset(
            args.data_dir, subset_, "val",   args.image_size,
            args.batch_size, preprocess_fn, training=False)

        optimizer = optimizers.Adam(learning_rate=1e-3)
        model.compile(
            optimizer=optimizer,
            loss="sparse_categorical_crossentropy",
            metrics=["accuracy"]
        )
        model.summary(print_fn=lambda x: logger.info(x))

        ckpt_path = os.path.join(args.output_dir, f"{subset_}_{args.backbone}_best.keras")
        csv_log_path = os.path.join(args.output_dir, f"{subset_}_{args.backbone}.csv")

        cbs = [
            callbacks.ModelCheckpoint(ckpt_path, monitor="val_accuracy", save_best_only=True, verbose=1), callbacks.ReduceLROnPlateau(monitor="val_loss", factor=0.5, patience=3, verbose=1),
            callbacks.CSVLogger(csv_log_path, append=True)
        ]

        logger.info(f"Phase 1: training head for {args.head_epochs} epochs")
        hist1 = model.fit(train_ds, validation_data=val_ds, epochs=args.head_epochs, callbacks=cbs, verbose=2)

        # ─── Phase 2: fine-tune backbone ───────────────────────────────
        logger.info("Phase 2: fine-tuning backbone")
        backbone.trainable = True
        for layer in backbone.layers[:-20]:
            layer.trainable = False

        model.compile(
            optimizer=optimizers.Adam(learning_rate=1e-4),
            loss="sparse_categorical_crossentropy",
            metrics=["accuracy"]
        )
        hist2 = model.fit(
            train_ds, validation_data=val_ds,
            initial_epoch=args.head_epochs,
            epochs=args.total_epochs,
            callbacks=cbs, verbose=2
        )

        final_path = os.path.join(args.output_dir, f"{subset_}_{args.backbone}_final.keras")
        model.save(final_path)
        logger.info(f"✅ Training complete. Model saved to: {final_path}")

        df = pd.read_csv(csv_log_path)
        epochs = df.index + 1

        plt.figure()
        plt.plot(epochs, df["accuracy"],        label="train_acc")
        plt.plot(epochs, df["val_accuracy"],    label="val_acc")
        plt.xlabel("Epoch"); plt.ylabel("Accuracy")
        plt.legend(); plt.title(f"{subset_} — {args.backbone} Accuracy")
        acc_plot = os.path.join(args.output_dir, f"{subset_}_{args.backbone}_accuracy.png")
        plt.savefig(acc_plot); plt.close()
        logger.info(f"Accuracy plot saved to: {acc_plot}")

        plt.figure()
        plt.plot(epochs, df["loss"],            label="train_loss")
        plt.plot(epochs, df["val_loss"],        label="val_loss")
        plt.xlabel("Epoch"); plt.ylabel("Loss")
        plt.legend(); plt.title(f"{subset_} — {args.backbone} Loss")
        loss_plot = os.path.join(args.output_dir, f"{subset_}_{args.backbone}_loss.png")
        plt.savefig(loss_plot); plt.close()
        logger.info(f"Loss plot saved to: {loss_plot}")

if __name__ == "__main__":
    main()
        
# python train.py --data_dir "/home/araviki/workbench/boilernet/model_train/imagenet" --output_dir "/home/araviki/workbench/boilernet/model_train/models" --backbone "mobilenet_v2" --image_size 224 --batch_size 32 --head_epochs 25 --total_epochs 55 --subset cats dogs fruits kitchen stationary
