import datetime
# import sys 
# print(f'{sys.path}')
import numpy

current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

def sayHello1():
  print("--------------------------------\n----------------Hello from sayHello1!-"+current_time+"\n")
  return 1

def sayHello2():
  print("--------------------------------\n----------------Hello from sayHello2!-"+current_time+"\n")
  return 2