#! /usr/bin/env python2
import logging

from control_center import create_app

if __name__ == '__main__':
    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] %(levelname)s in %(module)s:%("
        "lineno)s - %(funcName)15s() : %("
        "message)s"
    )
    app = create_app()
    app.run(debug=True, threaded=True, port=8001)
