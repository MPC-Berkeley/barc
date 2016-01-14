__author__ = 'brucewootton'

from data_api.models import LocalComputer

class AuthMiddleWare:
    def process_request(self, request):
        #
        # check that secret token matches local_computer id.
        #
        try:
            id = request.META['lc_id']
            uuid = request.META['lc_auth_token']
            local_computer = LocalComputer.objects.get(secret_uuid=uuid, id=id)
            request.user = local_computer.user
        except:
            pass

        return None
