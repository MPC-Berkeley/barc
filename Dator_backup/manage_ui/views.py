from django.shortcuts import render

__author__ = 'brucewootton'

def simple_view(request):
    return render(request, 'simple_view.html')


def root_view(request):
    return render(request, 'root_view.html')