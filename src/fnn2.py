import tensorflow as tf
import math
import numpy as np

class FullNeuralNetwork(object):
	def __init__(self):
		#Parameters
		self.perc_train = .7
		self.epochs = 100
		self.batch_size = 8
		self.learning_rate = .001
		self.input_size = 6
		self.hidden_size =6
		self.output_size = 3
		self.data_file = "test.txt"
	
 		#Placeholders
		self.x = tf.placeholder(tf.float32, shape=[self.batch_size,self.input_size])
		self.y = tf.placeholder(tf.float32, shape=[self.batch_size,self.output_size])
		self.dataset = []
		self.trainset = []
		self.testset = []

		#setup
		self._init_dataset()
		self._init_weights()
		self.logits = self._init_logits()
		self.loss = self._init_loss()
		self.optimizer = self._init_optimizer()

		#Set up instances
		init = tf.initialize_all_variables()
		self.sess = tf.Session()
		self.sess.run(init)

	def _init_dataset(self):
		f = open(self.data_file,'rb').read()
		flist = f.split("\n")
		for datapoint in flist:
			if datapoint != "":
				fsplit =  datapoint.split("], [")
				arm = fsplit[0][2:]
				ball = fsplit[1][:-2]
				arm = [float(a) for a in arm.split(",")]
				ball = [float(b) for b in ball.split(",")]
				self.dataset.append([arm+ball,ball])
				#print arm+ball
		num_train = int(len(self.dataset)*self.perc_train)
		self.trainset = self.dataset[0:num_train]
		self.testset = self.dataset[num_train:]
		

	def _init_weights(self):
		self.fcw1 = tf.Variable(tf.truncated_normal([self.input_size, self.output_size],
			stddev=1.0/math.sqrt(float(self.input_size))),
			name='weights')
		self.fcb1 = tf.Variable(tf.zeros([self.output_size]),
			name='biases')

	def _init_logits(self):
		self.hidden1 = tf.matmul(self.x,self.fcw1)+self.fcb1
		return self.hidden1

	def _init_loss(self):
		loss_l2 = tf.reduce_sum(tf.square(self.y-self.logits))
		return loss_l2

	def _init_optimizer(self):
		return tf.train.RMSPropOptimizer(self.learning_rate).minimize(self.loss)
	def get_train_batch(self):
		mybatch = []
		i1 = 0
		i2 = self.batch_size
		while i2 < len(self.trainset):
			mybatch.append(self.trainset[i1:i2])
			i1 += self.batch_size
			i2 += self.batch_size
		return mybatch
	def get_test_batch(self):
		mybatch = []
		i1 = 0
		i2 = self.batch_size
		while i2 < len(self.testset):
			mybatch.append(self.testset[i1:i2])
			i1 += self.batch_size
			i2 += self.batch_size
		return mybatch

	def _init_train(self):
		for epoch in range(self.epochs):
			av = 0
			b = 0
			for batch_xy in self.get_train_batch():
				batch_x = []
				batch_y = []
				for bxy in batch_xy:
					batch_x.append(bxy[0])
					batch_y.append(bxy[1])
				feed_dict = {self.x:batch_x,
					self.y:batch_y}
				_, loss_value = self.sess.run([self.optimizer, self.loss],
					feed_dict=feed_dict)	
				#print loss_value
				#print batch_x
				
				b += 1
				av += loss_value
			print "EPOCH: ",epoch , "AVERAGE LOSS:", av/float(b)

	def testing(self):
		for batch_xy in self.get_test_batch():
				batch_x = []
				batch_y = []
				for bxy in batch_xy:
					batch_x.append(bxy[0])
					batch_y.append(bxy[1])
				feed_dict = {self.x:batch_x}
				arm_guess = self.sess.run([self.logits], feed_dict=feed_dict)
		print batch_x
		print arm_guess
			

				

if __name__ == "__main__":
	fnn = FullNeuralNetwork()	
	fnn._init_train()
	print fnn.testing()
