from flask import Flask, render_template, request, jsonify, send_from_directory

import URDF_writer

app = Flask(__name__, static_folder='static', static_url_path='')

#load view_urdf.html
@app.route('/')
def index():
    return render_template('view_urdf.html')

#call URDF_writer.py to modify the urdf
@app.route('/changeURDF/', methods=['POST'])
def changeURDF():
    filename = request.form.get('module_name', 0)
    data = URDF_writer.main(filename)
    data = jsonify(data)
    return data

#call URDF_writer.py to remove the last module
@app.route('/removeModule/', methods=['POST'])
def remove():
    data = URDF_writer.remove_module()
    data = jsonify(data)
    return data

#upload on the server the /static folder
@app.route('/<path:path>')
def send_file(path):
    return send_from_directory(app.static_folder, path)


if __name__ == '__main__':
    app.run(debug=True, threaded=True) 
    # from gevent.pywsgi import WSGIServer
    # http_server = WSGIServer(('', 5000), app)
    # http_server.serve_forever()
