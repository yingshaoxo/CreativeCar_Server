from auto_everything.base import Super, Terminal
s = Super(username="root")
t = Terminal()

s.service("test", "./server_control.py")
print(t.run_command("sudo systemctl unmask test"))
