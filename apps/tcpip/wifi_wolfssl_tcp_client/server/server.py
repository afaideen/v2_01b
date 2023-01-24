
from flask import Flask

app = Flask(__name__)

@app.route("/post/test", methods=['POST','GET'])
def test():
    s = 'Hi! I am your server here.'
    return s

if __name__ == '__main__':
    # app.run(host="0.0.0.0", debug=True, port=5000)
    # socketio.run(app, debug=False, port=5000, host="0.0.0.0")
    ##use '0.0.0.0' to access the server from external ip
    app.run( host="0.0.0.0", port=5000, ssl_context = ('cert.pem', 'key.pem') )




    