#!/usr/bin/env python2
"""
Adapted from https://gist.github.com/bradmontgomery/2219997

Very simple HTTP server in python.
Usage::
    ./dummy-web-server.py [<port>]
Send a GET request::
    curl http://localhost
Send a HEAD request::
    curl -I http://localhost
Send a POST request::
    curl -d "foo=bar&bin=baz" http://localhost

If address in use: fuser -n tcp -k 8080
"""
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
import SocketServer
import rospy
from std_msgs.msg import Int32MultiArray


class S(BaseHTTPRequestHandler):

    def __init__(self, *args):
        rospy.init_node('https_bridge')
        self.pub = rospy.Publisher('rangefinder', Int32MultiArray, queue_size=10)
        BaseHTTPRequestHandler.__init__(self, *args)

    def _set_headers(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()

    def do_GET(self):
        ''' Expected data format: "/[drone index];[val 1];[val 2];...[val n]"
            Drone ordering is "alexa", "google", "siri", "clippy"
        '''
        vals = [float(x) for x in self.path[1:].split(';')]
        print(vals)
        print(self._set_headers())
        self.pub.publish(data=vals)
        self.wfile.write("<html><body><h1>hi!</h1></body></html>")

    def do_HEAD(self):
        self._set_headers()
        
    def do_POST(self):
        # Doesn't do anything with posted data
        self._set_headers()
        self.wfile.write("<html><body><h1>POST!</h1></body></html>")
        
def run(server_class=HTTPServer, handler_class=S, port=8080):
    server_address = ('', port)
    httpd = server_class(server_address, handler_class)
    httpd.allow_reuse_address = True
    print('Starting httpd...')
    rospy.on_shutdown(lambda:httpd.shutdown())
    httpd.serve_forever()

if __name__ == "__main__":
    from sys import argv

    if len(argv) == 2:
        run(port=int(argv[1]))
    else:
        run()