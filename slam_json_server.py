"""
This entire file is simply a set of examples. The most basic is to
simply create a custom server by inheriting tserver.ThreadedServer
as shown below in MyServer.
"""

import json
import jsocket
import logging
import Queue as queue
import time
import jsocket


class MyFactoryThread(jsocket.ServerFactoryThread):
	"""	This is an example factory thread, which the server factory will
		instantiate for each new connection.
	"""
	def __init__(self):
		super(MyFactoryThread, self).__init__()
		self.timeout = 2.0
		self.logger = logging.getLogger("slam_json_sever")
		self.q = queue.Queue()
	
	def _process_message(self, obj):
		""" virtual method - Implementer must define protocol """
		if obj != '':
			if obj['message'] == "new connection":
				self.logger.info("new connection.")
			else:
				self.logger.info("new message.")
				self.q.put(obj['message'])


class SlamJsonServer(object):
	def __init__(self):
		self.server = jsocket.ServerFactory(MyFactoryThread, address='0.0.0.0', port=5490)
		self.server.timeout = 10.0
		self.server.start()

	def get_data(self):
		obj = None
		for t in self.server._threads:
			if not t.q.empty():
				obj = json.loads(t.q.get())
				break
		return obj
		# m_t = obj['motor_ticks']
		# s_d = obj['scan_data']

	def stop(self):
		self.server.stop_all()
		self.server.stop()
		self.server.join()




