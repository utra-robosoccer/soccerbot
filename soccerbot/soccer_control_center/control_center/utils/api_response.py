class ApiResponse(object):
    @staticmethod
    def serialize_success(data):
        data = data or dict()
        data['status'] = 'success'
        return data

    @staticmethod
    def serialize_error(error_message, data=None):
        data = data or dict()
        data.update({
            'status': 'error',
            'error_message': error_message,
        })
        return data

    @staticmethod
    def is_error(data):
        if data.get('status') == 'error':
            return True
        return False
