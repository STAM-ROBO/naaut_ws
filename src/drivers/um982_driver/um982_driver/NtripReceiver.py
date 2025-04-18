import socket
import base64
import threading

class NtripReceiver():
    def __init__(self, _host, _port, _user, _password, _mountpoint, _queue):
        self.host=_host
        self.port=_port
        self.mountpoint=_mountpoint
        self.username=_user
        self.password=_password
        self.socket=None
        self.queue=_queue
        self.status = "inactive"
        http_thread = threading.Thread(target=self._connect, daemon=True)
        http_thread.start()

    def _connect(self):
        try:
            self.status = "connecting"
            #print(self.status)
            self.socket = socket.create_connection((self.host, self.port), timeout=5.0)     
            
            request_header = f"GET /{self.mountpoint} HTTP/1.0\r\n"
            request_header += f"Host: {self.host}\r\n"
            request_header += "User-Agent: NTRIPClient\r\n"

            if self.username and self.password:
                credentials = f"{self.username}:{self.password}"
                encoded_credentials = base64.b64encode(credentials.encode()).decode()
                request_header += f"Authorization: Basic {encoded_credentials}\r\n"
            request_header += "\r\n"
            
            self.socket.sendall(request_header.encode())
            response = self.socket.recv(4096).decode()
            if "ICY 200 OK" not in response:
                raise Exception(f"Failed to connect: {response}")
            
            self.status = "connected"
            #print(self.status)
                        
            cnt=0
            while True:
                data = self.socket.recv(4096)
                self.status = f"streaming chunk {cnt}"
                #print(self.status)
                if data:
                    cnt=cnt+1
                    self.queue.put(data)
                                                    
        except Exception as e:
            self.status = "error"
            #print(self.status)
            print(e)
