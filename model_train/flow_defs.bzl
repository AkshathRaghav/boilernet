def _define_or_default(name, default):
    """Returns a Bazel macro expression that uses --define or default."""
    return "$(D{})".format(name.upper()) if "$(D{})".format(name.upper()) else default

def training_flow(name):
    native.genrule(
        name = name,
        outs = ["train_done_marker.txt"],
        cmd = """
        mkdir -p model_train/output_checkpoints && \
        python3 model_train/train.py \
          --data_dir=model_train/full-imagenet/imagenet \
          --subset "$(SUBSET)" \
          --output_dir model_train/output_checkpoints \
          --backbone "$(BACKBONE)" \
          --image_size 224 \
          --batch_size "$(BATCH_SIZE)" \
          --head_epochs "$(HEAD_EPOCHS)" \
          --total_epochs "$(TOTAL_EPOCHS)" && \
        touch $@
        """,
        toolchains = [],
        visibility = ["//visibility:public"],
        substitutions = {
            "$(SUBSET)":  "$(Dsubset dogs)",
            "$(BACKBONE)": "$(Dbackbone resnet50)",
            "$(BATCH_SIZE)": "$(Dbatch_size 64)",
            "$(HEAD_EPOCHS)": "$(Dhead_epochs 5)",
            "$(TOTAL_EPOCHS)": "$(Dtotal_epochs 30)",
        },
    )

def conversion_flow(name):
    native.genrule(
        name = name,
        outs = ["conversion_done_marker.txt"],
        cmd = """
        mkdir -p model_train/output_headers && \
        python3 model_train/convert_header.py \
          --saved_model_dir=model_train/output_checkpoints \
          --data_dir=model_train/full-imagenet/imagenet \
          --subset "$(SUBSET)" \
          --tflite_path=model_train/output_headers/model.tflite \
          --header_path=model_train/output_headers/model.cc \
          --array_name=model_tflite \
          --image_size 32 && \
        touch $@
        """,
        toolchains = [],
        visibility = ["//visibility:public"],
        substitutions = {
            "$(SUBSET)": "$(Dsubset dogs)",
        },
    )
