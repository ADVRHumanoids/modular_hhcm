from flask import Flask, render_template, request, jsonify, send_from_directory


app = Flask(__name__, static_folder='static', static_url_path='')

@app.route('/')
def index():
    return render_template('view_urdf.html')

@app.route('/square/', methods=['POST'])
def square():
    num = str(request.form.get('module_name', 0))
    data = {'result': num}
    data = jsonify(data)
    return data

@app.route('/<path:path>')
def send_file(path):
    return send_from_directory(app.static_folder, path)


if __name__ == '__main__':
    app.run(debug=True)
