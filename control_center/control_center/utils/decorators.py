import errno
import socket
from functools import wraps
from xmlrpclib import Fault

from flask import jsonify, make_response

from control_center.utils.api_response import ApiResponse
from control_center.utils.utils import is_online


def required_online():
    def wrapper(f):
        @wraps(f)
        def wrapped(*args, **kwargs):
            host = kwargs.get('name')
            if not host:
                return make_json_response('name field not present', 400)
            if not is_online(host):
                return make_json_response('robot not online', 400)
            return f(*args, **kwargs)

        return wrapped

    return wrapper


def jsonify_response():
    def wrapper(f):
        @wraps(f)
        def wrapped(*args, **kwargs):
            retval = f(*args, **kwargs)
            if isinstance(retval, tuple):
                body, status_code = retval
            else:
                body = retval
                status_code = 200
            return make_json_response(body, status_code)

        return wrapped

    return wrapper


def make_json_response(data, status_code):
    if is_success_status_code(status_code):
        body = success_reponse(data)
    else:
        body = error_response(data)
    response = make_response(jsonify(body), status_code)
    response.headers['mimetype'] = 'application/json'
    return response


def is_success_status_code(code):
    return 200 <= code <= 299


def success_reponse(response):
    rv = dict()
    rv['status'] = 'success'
    rv['response'] = {'data': response}
    return rv


def error_response(response, data=None):
    rv = dict()
    data = data or dict()
    rv['status'] = 'error'
    rv['response'] = {'error': response, 'data': data}
    return rv


def handle_supervisor_error(func):
    @wraps(func)
    def handler(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Fault as error:
            return ApiResponse.serialize_error(error.faultString)
        except socket.error as e:
            if e.errno == errno.ECONNREFUSED:
                return ApiResponse.serialize_error('can not connect to xml server: ' + e.strerror)
            raise

    return handler
