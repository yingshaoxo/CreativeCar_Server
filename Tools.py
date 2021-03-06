#!/usr/bin/env /usr/bin/python3
from auto_everything.base import Python, Terminal
py = Python()
t = Terminal()

class Tools():
    def __clear(self):
        commands = """
sudo rm -fr king_chat.egg-info
sudo rm -fr dist
sudo rm -fr build
        """
        t.run(commands)

    def push(self, comment):
        self.__clear()

        t.run('git add .')
        t.run('git commit -m "{}"'.format(comment))
        t.run('git push origin')

    def pull(self):
        t.run("""
git fetch --all
git reset --hard origin/master
""")

    def service(self):
        from auto_everything.base import Super
        s = Super()
        s.service('test', 'server.py')

py.make_it_runnable()
py.fire(Tools)
