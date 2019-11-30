# 0.0.0.0:9958/car
# POST: json; like 'action=up, down, left, right, stop'
import cv2
import numpy as np
import math
from datetime import datetime
import os

from flask import Flask, request, render_template, redirect
from auto_everything.base import Terminal
t = Terminal()
print(t.run_command("vcgencmd measure_temp"))

app = Flask(__name__)

State = False
robot_state = False

try:
    cap = cv2.VideoCapture(0)
    w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    cap.set(3, w)
    cap.set(4, h)
except Exception as e:
    print(e)


@app.route('/car', methods=['POST', 'GET'])
def handle_car_request():
    global State
    if request.method == 'POST':
        if request.is_json:
            content = request.get_json()
            print(content)
            if ('action' in content):
                action_name = content['action']
                if State == False:
                    t.run_py("server.py")
                    # t.run_py("tracking.py")
                    print('server has started')
                    State = True
                else:
                    t.kill("server.py")
                    t.run_py("clean.py")
                    # t.kill("tracking.py")
                    print('server has stoped')
                    State = False

            return "ok"

    elif request.method == "GET":
        return "havn't yet"


@app.route('/', methods=['GET'])
def handle_web_request():
    jpg_file_list = list(reversed([i for i in sorted(filter(os.path.isfile, ["./static/"+f for f in os.listdir("./static")]), key=os.path.getmtime) if i[-4:] == ".jpg"]))
    #print(f"origin: {jpg_file_list}")

    file_to_remove = jpg_file_list[10:]
    #print(f"file_to_remove: {file_to_remove}")
    jpg_file_list = jpg_file_list[:10]
    #print(f"remaining: {jpg_file_list}")
    for file_ in file_to_remove:
        os.remove(f"{file_}")

    my_items = []
    for index, file_ in enumerate(jpg_file_list):
        my_items.append({
            "index":  index,
            "name": file_[9:]
        })

    print(f"my_items: {my_items}")
    return render_template("index.html", items=my_items)


@app.route('/startorstop', methods=['POST'])
def start_or_stop():
    global robot_state

    if request.method == 'POST':
        if robot_state == False:
            t.run_py("robot.py")
            print('robot has started')
            robot_state = True
        else:
            t.kill("robot.py", force=True)
            t.run_py("reset_pins.py")
            print('robot has stoped')
            robot_state = False

    # return render_template("index.html")
    return ""


def white_balance(img):
    result = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    avg_a = np.average(result[:, :, 1])
    avg_b = np.average(result[:, :, 2])
    result[:, :, 1] = result[:, :, 1] - ((avg_a - 128) * (result[:, :, 0] / 255.0) * 1.1)
    result[:, :, 2] = result[:, :, 2] - ((avg_b - 128) * (result[:, :, 0] / 255.0) * 1.1)
    result = cv2.cvtColor(result, cv2.COLOR_LAB2BGR)
    return result


def effect_of_whitening(frame, whiten_level=5.0):
    assert 1 <= whiten_level <= 5, "whiten_level must belongs to [1, 5]"

    magic_number = 0.003921
    a = math.log(whiten_level)
    new_frame = (255 * (np.log((frame * magic_number) *
                               (whiten_level-1) + 1) / a)).astype(np.uint8)
    return new_frame


@app.route('/takepicture', methods=['GET'])
def take_picture():
    global cap
    try:
        # You may want to add white_balance here!

        if cap.isOpened():
            _, frame = cap.read()
            # cap.release()  # releasing camera immediately after capturing picture
            if _ and frame is not None:
                #frame = effect_of_whitening(frame)
                #frame = effect_of_whitening(frame)
                cv2.imwrite(f'static/{datetime.now().strftime("%s")}.jpg', frame)
    except Exception as e:
        print(e)
    return redirect("/")


if __name__ == '__main__':
    app.run(host="0.0.0.0", port=9999, debug=False)
