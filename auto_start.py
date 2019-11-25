from auto_everything.base import Super, Terminal
s = Super(username="root")
t = Termianl()

s.service("test", "./server_control.py")
t.run_command("sudo systemctl unmask test")

