#!/usr/bin/python  
# -*- coding: utf-8 -*-  
# python 二叉树  
      
def create_btree( e ):  
 root = {"a":e[0], "left":None, "right":None}  
 for i in e:  
  insert(root, i)  
 return root  
      
def insert(root, i):  
 if i==root["a"]:  
  return  
 if i<root["a"]:  
  if root["left"] != None:  
   insert(root["left"], i)  
  else:  
   root["left"] = {"a":i, "left":None, "right":None}  
 if i>root["a"]:  
  if root["right"] != None:  
   insert(root["right"], i)  
  else:  
   root["right"] = {"a":i, "left":None, "right":None}  
 if i==root['a']:
  print 'equal'
  
def pre_order(root):  
 if root==None: 
  return  
 pre_order( root["left"] )  
 pre_order( root["right"] )
 print   
 print 'tree: ',root 
 
 
def found(root, i):
 if i <root['a']:
  if root['left']==None:
        return None
  else:
      return found(root['left'],i)
 elif i >root['a']:
  if root['right']==None:
        return None
  else:  
   return found(root['right'],i)
 else:
  return root['a']
 
if __name__ == '__main__':  
 l = [ 4,1,2,3, 2, -1, 5, 8 ] 
 root =create_btree(l) 
 pre_order(root) 
 print found(root,-10) 


 
