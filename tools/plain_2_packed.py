import argparse
import os

header_magic = [0x4c, 0x48, 0x54]
model_type_map = {
    'fomo': 1,
    'pfld': 2,
    'yolo': 3
}

def parse_args():
    parser = argparse.ArgumentParser(description='Convert plain TFLite model to EdgeLab packed TFLite model')
    parser.add_argument('model', help='TFLite model file', type=str)
    parser.add_argument('id', help='A unique ID for packed binary model, range [1, 15]', type=int)
    parser.add_argument('type', help='TFLite model type', type=str, choices=model_type_map.keys())
    args = parser.parse_args()

    assert(os.path.splitext(args.model)[1] == '.tflite')
    assert(args.id in range(1, 16))

    return args

if __name__ == '__main__':
    args = parse_args()

    model_path = os.path.join(os.getcwd(), args.model)
    with open(model_path, 'rb') as model:
        data = model.read()
    
    header = bytearray(header_magic) + (args.id << 4 | model_type_map[args.type]).to_bytes(1, 'big') + (len(data) << 8).to_bytes(4, 'big')
    output = header + data

    output_file = '{}.bin'.format(os.path.splitext(model_path)[0])
    with open(output_file, 'wb') as binary:
        binary.write(output)

    print('Convert Succeed ->\n',
          f'\tmodel:\t {model_path}\n',
          f'\tsize:\t {hex(len(data))}\n',
          f'\tid:\t {args.id}\n',
          f'\ttype:\t {args.type}\n'
          f'\toutput:\t {output_file}')
