from flask import Flask, render_template, jsonify, send_from_directory
from flask_socketio import SocketIO, emit
import json
import os
import threading
import time
from datetime import datetime
import glob
import logging

class WebInterface:
    def __init__(self, port=5000):
        self.app = Flask(__name__, template_folder='../web_templates', static_folder='../web_static')
        self.app.config['SECRET_KEY'] = 'robot_exploration_secret'
        
        # Disable Flask request logging
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)
        
        self.socketio = SocketIO(self.app, cors_allowed_origins="*", logger=False, engineio_logger=False)
        self.port = port
        
        # Data storage
        self.conversation_log = []
        self.robot_status = {
            'active': False,
            'current_action': 'Idle',
            'position': {'x': 0, 'y': 0, 'heading': 0},
            'last_update': datetime.now().isoformat()
        }
        
        # Ensure directories exist
        os.makedirs('../snapshots', exist_ok=True)
        os.makedirs('../web_templates', exist_ok=True)
        os.makedirs('../web_static', exist_ok=True)
        
        self.setup_routes()
        
    def setup_routes(self):
        @self.app.route('/')
        def index():
            return render_template('index.html')
            
        @self.app.route('/api/status')
        def get_status():
            return jsonify(self.robot_status)
            
        @self.app.route('/api/conversation')
        def get_conversation():
            return jsonify(self.conversation_log)
            
        @self.app.route('/api/snapshots')
        def get_snapshots():
            # Get list of snapshot files
            snapshot_files = glob.glob('../snapshots/*.jpg')
            snapshots = []
            for file_path in sorted(snapshot_files, key=os.path.getmtime, reverse=True):
                filename = os.path.basename(file_path)
                timestamp = os.path.getmtime(file_path)
                snapshots.append({
                    'filename': filename,
                    'timestamp': datetime.fromtimestamp(timestamp).isoformat(),
                    'url': f'/snapshots/{filename}'
                })
            return jsonify(snapshots)
            
        @self.app.route('/snapshots/<filename>')
        def serve_snapshot(filename):
            return send_from_directory('../snapshots', filename)
            
        @self.socketio.on('connect')
        def handle_connect():
            print('Client connected to web interface')
            emit('status_update', self.robot_status)
            
        @self.socketio.on('disconnect')
        def handle_disconnect():
            print('Client disconnected from web interface')
    
    def log_message(self, message_type, content, metadata=None):
        """Log a message to the conversation log"""
        log_entry = {
            'timestamp': datetime.now().isoformat(),
            'type': message_type,  # 'user', 'agent', 'tool', 'system', 'debug'
            'content': content,
            'metadata': metadata or {}
        }
        self.conversation_log.append(log_entry)
        
        # Emit to connected clients
        self.socketio.emit('new_message', log_entry)
        
        # Keep only last 1000 messages
        if len(self.conversation_log) > 1000:
            self.conversation_log = self.conversation_log[-1000:]
    
    def log_debug(self, content, metadata=None):
        """Convenience method to log debug messages"""
        self.log_message('debug', content, metadata)
    
    def update_status(self, **kwargs):
        """Update robot status"""
        self.robot_status.update(kwargs)
        self.robot_status['last_update'] = datetime.now().isoformat()
        
        # Emit to connected clients
        self.socketio.emit('status_update', self.robot_status)
    
    def log_snapshot(self, snapshot_path, description):
        """Log a snapshot with description"""
        filename = os.path.basename(snapshot_path)
        
        # Count how many snapshots we've taken
        snapshot_count = len([msg for msg in self.conversation_log if msg.get('type') == 'snapshot']) + 1
        
        self.log_message('snapshot', f'Snapshot {snapshot_count} taken: {description}', {
            'snapshot_filename': filename,
            'snapshot_url': f'/snapshots/{filename}',
            'snapshot_count': snapshot_count
        })
        
        # Emit snapshot update
        self.socketio.emit('new_snapshot', {
            'filename': filename,
            'timestamp': datetime.now().isoformat(),
            'url': f'/snapshots/{filename}',
            'description': description,
            'snapshot_count': snapshot_count
        })
    
    def run(self, debug=False):
        """Run the web server"""
        print(f"🌐 Starting web interface on http://localhost:{self.port}")
        # Disable access logging
        self.socketio.run(self.app, host='0.0.0.0', port=self.port, debug=False, use_reloader=False)
    
    def run_threaded(self, debug=False):
        """Run the web server in a separate thread"""
        thread = threading.Thread(target=self.run, kwargs={'debug': debug})
        thread.daemon = True
        thread.start()
        return thread
