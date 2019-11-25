# utility.py
from threading import Lock
from datetime import datetime

print_lock = Lock()

def log_string(user_msg):
    """ Prints the desired string to the shell, preceded by the date and time.
    """
    with print_lock:
        print(datetime.now().strftime('%H.%M.%S.%f') + " " + user_msg)
