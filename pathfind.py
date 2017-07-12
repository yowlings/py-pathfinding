import cv2 as cv
import numpy as np
import time
import Queue
from matplotlib import pyplot as plt

class pathfind():
	"""docstring for self.pathfind"""
	def __init__(self, graph,robot_radius,start,goal):
		self.graph = graph
		self.robot_radius = robot_radius
		self.start = start
		self.goal = goal
		self.orimg = cv.imread(self.graph)
		self.img = cv.imread(self.graph, 0)
		self.columns, self.rows = self.img.shape
	def threshold(self):
		for i in range(self.columns):
			for j in range(self.rows):
				if self.img[i,j]>128:
					self.img[i,j]=255
				else:
					self.img[i,j]=0
	def inflate(self):
		for i in range(self.columns):
			for j in range(self.rows):
				if self.img[i,j]==0:
					for u in [-2,-1,0,1,2]:
						for v in [-2,-1,0,1,2]:
							if(u!=0 and v!=0):
								if (((i+u)>=0)&((i+u)<self.columns)&((j+v)>=0)&((j+v)<self.rows)):
									self.orimg[i+u,j+v,1]=128
									self.img[i+u,j+v]=128
	def draw(self):
		# cv.line(self.orimg, start, goal, (255,0,0),1)
		cv.circle(self.orimg, (self.start[1],self.start[0]), 2, (0,255,0),2)
		cv.circle(self.orimg, (self.goal[1],self.goal[0]), 2, (0,0,255),2)
		font = cv.FONT_HERSHEY_SIMPLEX
		cv.putText(self.orimg, 'PathFinding', (30,200), font, 1, (0,0,0), 2)
	def get_neighbors(self,current):
		neighbors = []
		if(current[0]+1<self.columns and self.img[current[0]+1,current[1]]==255):
			neighbors.append((current[0]+1, current[1]))
		if(current[0]-1>=0 and self.img[current[0]-1,current[1]]==255):
			neighbors.append((current[0]-1, current[1]))
		if(current[1]-1>=0 and self.img[current[0],current[1]-1]==255):
			neighbors.append((current[0], current[1]-1))
		if(current[1]+1<self.rows and self.img[current[0],current[1]+1]==255):
			neighbors.append((current[0], current[1]+1))
		return neighbors
	def heuristic(self,a,b):
		return abs(a[0] - b[0]) + abs(a[1] - b[1])
	def draw_path(self):
		current = self.goal
		self.path = [current]
		while current != self.start:
			cv.line(self.orimg, (current[1],current[0]), (self.came_from[current][1],self.came_from[current][0]), (255,0,0),1)
			current = self.came_from[current]
			self.path.append(current)
		self.path.append(self.start)
		self.path.reverse()
	def pre_process(self):
		self.threshold()
		self.inflate()
	def post_process(self):
		self.draw()
		self.draw_path()
		self.orimg[self.start[0],self.start[1],2] = 0
		self.orimg[self.start[0],self.start[1],1] = 0
		self.orimg[self.start[0],self.start[1],0] = 255
	def expand(self):
		frontier = Queue.Queue()
		frontier.put(self.start)
		visited = {}
		visited[start] = True
		while not frontier.empty():
			current = frontier.get()
			neighbors = self.get_neighbors(current)
			while not neighbors.empty():
				next = neighbors.get()
				if next not in visited:
					frontier.put(next)
					visited[next] = True
					self.orimg[next[0],next[1],2] = 128
	def breadth_first(self):
		frontier = Queue.Queue()
		frontier.put(self.start)
		self.came_from = {}
		self.came_from[self.start] = None
		# self.orimg[self.start[0],self.start[1],2] = 50
		while not frontier.empty():
			current = frontier.get()
			if current == self.goal:
				break
			neighbors = self.get_neighbors(current)
			for next in neighbors:
				if next not in self.came_from:
					frontier.put(next)
					self.came_from[next] = current
					self.orimg[next[0],next[1],2] = 0
					cv.imshow('orimg', self.orimg)
					cv.waitKey(1)
	def dijkstra(self):
		frontier = Queue.PriorityQueue()
		frontier.put((0, self.start))
		self.came_from = {}
		self.cost_so_far = {}
		self.came_from[self.start] = None
		self.cost_so_far[self.start] = 0
		while not frontier.empty():
			current = frontier.get()[1]
			if current == self.goal:
				break
			neighbors = self.get_neighbors(current)
			for neighbor in neighbors:
				new_cost = self.cost_so_far[current] + 1
				# print new_cost
				if neighbor not in self.cost_so_far or new_cost < self.cost_so_far[neighbor]:
					self.cost_so_far[neighbor] = new_cost
					priority = new_cost
					frontier.put((priority, neighbor))
					# print frontier
					self.came_from[neighbor] = current
					self.orimg[neighbor[0],neighbor[1],0] = 0
					self.orimg[neighbor[0],neighbor[1],1] = 0
					# cv.imshow('orimg', self.orimg)
					# cv.waitKey(1)

	def astar(self):
		frontier = Queue.PriorityQueue()
		frontier.put((0, self.start))
		self.came_from = {}
		cost_so_far = {}
		self.came_from[start] = None
		cost_so_far[start] = 0
		while not frontier.empty():
			current = frontier.get()[1]
			if current == goal:
				break
			neighbors = self.get_neighbors(current)
			for next in neighbors:
				new_cost = cost_so_far[current] + 1
				if next not in cost_so_far or new_cost < cost_so_far[next]:
					cost_so_far[next] = new_cost
					priority = new_cost + self.heuristic(self.goal, next)
					frontier.put((priority, next))
					self.came_from[next] = current
					self.orimg[next[0],next[1],0] = 0
					self.orimg[next[0],next[1],1] = 0
					# cv.imshow('orimg', self.orimg)
					# cv.waitKey(1)



graph='office.jpg'
start = (176,42)
goal = (200,130)

d1 = pathfind(graph,1,start,goal)
d1.pre_process()
begin = time.time()
d1.astar()
end = time.time()
print d1.columns,d1.rows

d1.post_process()

print end - begin
cv.imshow('orimg', d1.orimg)
cv.imshow('img', d1.img)
cv.waitKey(0)

