export FLASK_APP=server_control
export FLASK_ENV=development
flask run -h "0.0.0.0" -p 9999

# sudo crontab -e
# @reboot cd /home/pi/car && nice -n -20 python3 /home/pi/car/server_control.py &
