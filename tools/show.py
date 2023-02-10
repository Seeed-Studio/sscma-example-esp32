from serial import Serial
from serial.tools.list_ports import comports
import base64
from PIL import Image
import numpy as np
import cv2
import math


def calculate(start_x, start_y, end_x, end_y, center_x, center_y, pointer_x, pointer_y, range):
    pi = np.pi

    # assert math.sqrt(pow(abs(center_x - start_x), 2) + pow(abs(center_y - start_y), 2)) == \
    #        math.sqrt(pow(abs(center_x - end_x), 2) + pow(abs(center_y - end_y), 2)), \
    #         'the side length must be equal!'

    cneter_to_start = math.sqrt(
        pow(abs(center_x - start_x), 2) + pow(abs(center_y - start_y), 2))
    center_to_end = math.sqrt(
        pow(abs(center_x - end_x), 2) + pow(abs(center_y - end_y), 2))
    start_to_end_side = math.sqrt(
        pow(abs(end_x - start_x), 2) + pow(abs(end_y - start_y), 2))
    theta = np.arccos((pow(cneter_to_start, 2) + pow(center_to_end, 2) - pow(start_to_end_side, 2)
                       ) / (2 * cneter_to_start * center_to_end))  # cosθ=(a^2 + b^2 - c^2) / 2ab
    theta = 2 * pi - theta

    # determine center in which side of line from start to pointer
    A, B, C = center_y - start_y, start_x - \
        center_x, (center_x * start_y) - (start_x * center_y)
    D = A * pointer_x + B * pointer_y + C  # linear function: Ax+By+C=0

    start_to_pointer_side = math.sqrt(
        pow(abs(pointer_x - start_x), 2) + pow(abs(pointer_y - start_y), 2))
    center_to_pointer_side = math.sqrt(
        pow(abs(pointer_x - center_x), 2) + pow(abs(pointer_y - center_y), 2))
    theta1 = np.arccos((pow(cneter_to_start, 2) + pow(center_to_pointer_side, 2) -
                       pow(start_to_pointer_side, 2)) / (2 * cneter_to_start * center_to_pointer_side))

    theta1 = theta1 if D >= 0 else (2 * pi - theta1)

    if theta1 > theta:
        return None
    else:
        out = range * (theta1 / theta)
        return out


def base64img(width, height, channel, img):

    format = 'RGB'

    if channel == 3:
        format = 'RGB'
    elif channel == 1:
        format = 'L'

    try:
        imgdata = base64.b64decode(img)
        img = Image.frombytes(format, (width, height), imgdata)
        img = np.array(img)
        img = np.fliplr(img)
        return img
    except:
        return None


if __name__ == '__main__':

    ser = Serial('COM6', 2000000, timeout=2000)

    # text = 'AT+RUNIMPULSEDEBUG=y\r\n'
    # result = ser.write(text.encode("utf8"))

    data = ''
    width = 0
    height = 0
    channel = 0
    model = ''

    while True:
        rev_num = ser.inWaiting()
        if rev_num:
            s = ser.read(rev_num)
            data += str(s, encoding='utf-8')

            if data.find('End output') > -1:
                currentMsg = data.split('End output')[0]
                img = None

                line = currentMsg.split('\n')

                for i in range(len(line)):
                    if line[i].find('Format: ') > -1:
                        print(line[i])
                        fmt = eval(line[i].split('Format: ')
                                   [1].replace('\r', ''))
                        width = fmt['width']
                        height = fmt['height']
                        channel = fmt['channels']
                        model = fmt['model']

                    if line[i].find('Framebuffer: ') > -1:
                        fb = line[i].split('Framebuffer: ')[
                            1].replace('\r', '')
                        img = base64img(240, 240, 1, fb)

                    if line[i].find('Predictions ') > -1:
                        print(line[i].split('Predictions ')
                              [1].replace('\r', ''))

                    if line[i].find('    ') > -1:
                        print(line[i])
                        if model == 'fomo':
                            if line[i].find('No objects found') <= -1:
                                label = line[i].split('(')[0].strip()
                                c = line[i].split('(')[1].split(')')[0]
                                x = int(line[i].split('x: ')[1].split(',')[0])
                                y = int(line[i].split('y: ')[1].split(',')[0])
                                w = int(line[i].split('width: ')
                                        [1].split(',')[0])
                                h = int(line[i].split('height: ')
                                        [1].split(' ]')[0])
                                cv2.circle(img, (x, y), w, (0, 0, 255), 1)
                                cv2.putText(img, label + ':' + c,
                                            (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)
                        elif model == 'meter':
                            if line[i].find('No objects found') <= -1:
                                label = line[i].split('(')[0].strip()
                                c = line[i].split('(')[1].split(')')[0]
                                x = int(line[i].split('x: ')[1].split(',')[0])
                                y = int(line[i].split('y: ')[1].split(' ]')[0])

                if type(img) != type(None):
                    img = cv2.resize(img, dsize=(240, 240))
                    cv2.imshow('img', img)
                    cv2.waitKey(1)
                    img = None

                data = data.split('End output')[1]

            if data.find('Begin output') > -1:
                data = data.split('Begin output')[1]
