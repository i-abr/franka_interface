

import select
import lcm
from exlcm import env_state


def my_handler(channel, data):
    msg = env_state.decode(data)
    print(msg.object_position)

lc = lcm.LCM()
lc.subscribe('env_state_pub', my_handler)

timeout = 0.1

while True:
    rfds, wfds, efds = select.select([lc.fileno()], [], [], timeout)
    if rfds:
        lc.handle()
    else:
        print('waiting for message ...')
