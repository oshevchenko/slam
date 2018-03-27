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
		self.f_s_d = open("scan_data.txt", "w", 1000000)
		self.f_m_t = open("motor_ticks.txt", "w", 1000000)

	def write_scan_data(self, line_header, scan_data):
		print >> self.f_s_d, line_header,
		# for s in scan_data:
		# 	print >> self.f_s_d, "%.1f" % s,
		print >> self.f_s_d

	def _process_message(self, obj):
		""" virtual method - Implementer must define protocol """
		if obj != '':
			if obj['message'] == "new connection":
				self.logger.info("new connection.")
			else:
				self.logger.info("new message.")
				data = json.loads(obj['message'])
				m_t = data['motor_ticks']
				s_d = data['scan_data']

				print >> self.f_s_d, "S 0 0",
				for s in s_d:
					print >> self.f_s_d, "%d" % s,
				print >> self.f_s_d
				print(m_t)
				print >> self.f_m_t, "M 1 %d 3 4 5 %d 7 8 9" % (m_t[0], m_t[1]),
				print >> self.f_m_t

				# print >> self.f_m_t, "M 0",
				# for m in m_t:
				# 	print >> self.f_m_t, "%d" % m,
				# print >> self.f_m_t

				data_for_queue=tuple((m_t, s_d))
				self.q.put(data_for_queue)


class SlamJsonServer(object):
	def __init__(self):
		self.server = jsocket.ServerFactory(MyFactoryThread, address='0.0.0.0', port=5490)
		self.server.timeout = 10.0
		self.server.start()

	def get_data(self):
		data = None
		for t in self.server._threads:
			if not t.q.empty():
				data = t.q.get()
				break
		return data
		# m_t = obj['motor_ticks']
		# s_d = obj['scan_data']

	def stop(self):
		for t in self.server._threads:
			t.f_s_d.close()
			t.f_m_t.close()

		self.server.stop_all()
		self.server.stop()
		self.server.join()




