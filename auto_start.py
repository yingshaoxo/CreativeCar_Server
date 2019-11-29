from auto_everything.base import Super, Terminal
s = Super(username="root")
t = Terminal()

s.service("test", "./server_control.py")
print(t.run_command("sudo systemctl unmask test"))

"""
OR

# sudo crontab -e
# @reboot cd /home/pi/car && nice -n -20 python3 /home/pi/car/server_control.py &
"""
