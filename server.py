# 0.0.0.0:9958/car
# POST: json; like 'action=up, down, left, right, stop'
from car import Car
car = Car()

#car.get_away_from_obstacles()
#car.stop_camera()
#car.start_camera()

from flask import Flask, request
app = Flask(__name__)

@app.route('/car', methods=['POST', 'GET'])
def handle_car_request():
    if request.method == 'POST':
        if request.is_json:
            content = request.get_json()
            if ('action' in content) and ('speed' in content):
                car.action(action_name=content['action'], speed=int(content['speed']), time=0)
            if 'camera_action' in content:
                car.camera_action(action_name=content['camera_action'])
            return "ok"

    elif request.method == "GET":
        return "havn't yet"

if __name__ == '__main__':
   app.run(host="0.0.0.0", port=9958, debug=False)
