# 0.0.0.0:9958/car
# POST: json; like 'action=up, down, left, right, stop'
from auto_everything.base import Terminal
t = Terminal()

from flask import Flask, request
app = Flask(__name__)

State = False

@app.route('/car', methods=['POST', 'GET'])
def handle_car_request():
    global State
    if request.method == 'POST':
        if request.is_json:
            content = request.get_json()
            if ('action' in content):
                action_name = content['action']
                if State == False:
                    t.run_py("server.py")
                    print('server has started')
                    State = True
                else:
                    t.kill("server.py")
                    print('server has stoped')
                    State = False

            return "ok"

    elif request.method == "GET":
        return "havn't yet"


if __name__ == '__main__':
    app.run(host="0.0.0.0", port=9999, debug=False)
