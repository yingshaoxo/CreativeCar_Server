from auto_everything.base import Super

s = Super(username="root")

s.service("test", "./server_control.py")
