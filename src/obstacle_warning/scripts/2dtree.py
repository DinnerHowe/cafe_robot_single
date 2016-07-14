#!/usr/bin/python  
# -*- coding: utf-8 -*-  
# python 二叉树  
      
def create_btree( e ):  
 root = {"a":e, "left":None, "right":None}  
 depth=0
 for i in e:  
  insert(root, i, depth)
 return root  
      
def insert(root, i, depth):
 depth+=1
 if depth<3 :
  if i==4:  
   return
  
  if i<4:  
   if root["left"] != None:  
    insert(root["left"], i, depth)  
   else:  
    root["left"] = {"a":[], "left":None, "right":None}  
   
  if i>4:  
   if root["right"] != None:  
    insert(root["right"], i, depth)  
   else:  
    root["right"] = {"a":[], "left":None, "right":None}  
 else:
  pass
  
def loader(root, i):
 if root["left"] == None and root['right']==None:  
  root['a'].append(i) 
 if root["left"] != None and root['right']==None:  
  loader(root["left"], i)      
 if root["left"] == None and root['right']!=None:  
  loader(root["right"], i)     
 if root["left"] != None and root['right']!=None:  
  if i<4: 
   loader(root["left"], i)  
  if i>=4:
   loader(root["right"], i)
  
 
 
def found(root, i):
 if i <4:
  if root['left']==None:
   if i in root['a']:
    return root['a']
   else:
    return None
  else:
   return found(root['left'],i)  
 elif i >=4:
  if root['right']==None:
   if i in root['a']:
    return root['a']
   else:
    return None
  else:  
   return found(root['right'],i)
 else:
  print 'error'

  
if __name__ == '__main__':  
 l = [ 4,1,2,3, 2, -1, 5, 8 ] 
 root =create_btree(l)
   
 for i in l:
  loader(root, i)
  
 number=4
 print 'tree: ',root 
 print '%s is in list'%number, found(root,number) 


 
