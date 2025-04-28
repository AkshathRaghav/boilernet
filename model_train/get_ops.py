# list_ops.py
import pathlib
import flatbuffers
# import the generated Python modules for the TFLite schema
import tflite.Model as Model
import tflite.BuiltinOperator as BuiltinOp

def list_ops(tflite_file):
    buf = pathlib.Path(tflite_file).read_bytes()
    model = Model.GetRootAsModel(buf, 0)

    ops = set()
    for i in range(model.OperatorCodesLength()):
        op_code = model.OperatorCodes(i)
        builtin = op_code.BuiltinCode()
        if builtin == BuiltinOp.CUSTOM:
            name = op_code.CustomCode().decode('utf-8')
        else:
            # get the enum name for this builtin code
            name = BuiltinOp.EnumName(builtin)
        ops.add(name)
    print("Operators in model:")
    for name in sorted(ops):
        print(" ", name)

if __name__ == "__main__":
    import sys
    list_ops(sys.argv[1])
