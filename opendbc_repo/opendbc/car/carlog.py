import os
import logging

# set up logging
LOGPRINT = os.environ.get('LOGPRINT', 'INFO').upper()
carlog = logging.getLogger('carlog')
carlog.setLevel(LOGPRINT)
carlog.propagate = False

handler = logging.StreamHandler()
handler.setFormatter(logging.Formatter('%(message)s'))
carlog.addHandler(handler)

# Records sent here are wired directly to swaglog IPC by card.py. Keeping this
# logger handler-free and non-propagating prevents research diagnostics from
# reaching stderr/tmux while still allowing logmessaged to persist them.
researchlog = logging.getLogger('carlog.research')
researchlog.setLevel(logging.DEBUG)
researchlog.propagate = False
