from tkinter import *
import time
import threading


class ButtonVertex():
    """
    Creates a button for each grid location. Buttons can be clicked on to make them walls or start and end points.
    """
    def __init__(self, master, position):
        self.button_obj = Button(master, command = self._performStep, width=2, height=1, bg='papaya whip')
        self.position = position
        self.adjacent = []
        self.wall_changed = False
        self.wall = 0
        self.start = 0
        self.goal = 0
            
    def _performStep(self):
        # The 'NEXT' button cycles through these steps and it dictates what should happen to a node when it is clicked on.
        global step
        if step == 0:
            self.createStart()
        elif step == 1:
            self.createGoal()
        elif step == 2:
            self.changeWall()
        else:
            pass
        
    def mouse_entered(self):
        # This function allows for click and drag creation of walls.
        if self.wall_changed == False:
            self.changeWall()
            self.wall_changed = True
        
    def createStart(self):
        # Assigns the start position and colors the start red.
        global start
        global vertexList
        start = self.position
        self.start = 1
        self.button_obj.configure(bg='red')
        
        for node in vertexList.values():
            if node.position != start:
                node.start = 0
                node.button_obj.configure(bg='papaya whip')
        
    def createGoal(self):
        # Assigns the goal position and colors the goal blue.
        global goal
        global start
        global vertexList
        if self.start != 1:

            goal = self.position
            self.goal = 1
            self.button_obj.configure(bg='blue')

            for node in vertexList.values():
                if (node.position != goal) and (node.position != start):
                    node.goal = 0
                    node.button_obj.configure(bg='papaya whip')

    def changeWall(self):
        # Creates grey walls.
        if self.start == 0 and self.goal == 0:
            self.wall = (self.wall+1)%2
            if self.wall == 0:
                self.button_obj.configure(bg='papaya whip')
            else:
                self.button_obj.configure(bg='grey')
        
    def findAdjacents(self):
        # This adds all adjacent nodes that are not walls to a node's adjacency list.
        global start
        for node_position in vertexList:
            if abs(node_position[0]-self.position[0])<=1 and abs(node_position[1]-self.position[1])<=1:
                if vertexList[node_position].wall == 0 and node_position != (self.position or start):
                    dist = round(((node_position[0]-self.position[0])**2+(node_position[1]-self.position[1])**2)**0.5, 3)
                    self.adjacent.append((dist, node_position))
        return self.adjacent
    
    def __str__(self):
        return f'Vector {self.position}'



def nextStep():
    # This changes the label on the GUI dependant on the current step. When the third step is reached, it activates the BFS.
    global step
    global label
    step+=1
    
    if step == 1:
        label.configure(text='Now select the goal!')
    elif step == 2:
        label.configure(text='Now make some walls.')
    else:
        
        global start
        global goal
        label.configure(text='Finding the best route...')
        path = BFS(start, goal)
        if path != None:
            label.configure(text='Found a path!')
            colorPath(path)
        else:
            label.configure(text='I was not able to find a path :(')



def mouse_up(event):
    # <B1-Motion> constantly returns a value. This makes sure the buttons aren't continuously triggered.
    for button in vertexList:
        vertexList[button].wall_changed = False

        
        
def mouse_motion(event):
    global frame1
    global step
    if step == 2:
        for button in vertexList:
            if frame1.winfo_containing(event.x_root, event.y_root) is vertexList[button].button_obj:
                vertexList[button].mouse_entered()
                root.update()
                
                
                
def colorPath(path):
    for node in path:
        vertexList[node].button_obj.configure(bg='green')



def BFS(start, goal):
    global root
    current = start
    path = [start]
    goal_dist = round(((start[0]-goal[0])**2+(start[1]-goal[1])**2)**0.5, 2)
    queue = {goal_dist:[(current, path)]}
    
    counter = 0
    
    while queue != [] and counter<100:
        min_dist = min(queue)
        (current, path) = queue[min_dist].pop(0)
        if queue[min_dist] == []:
            del queue[min_dist]
        vertexList[current].findAdjacents()
        
        for nxt in vertexList[current].adjacent:

            nxt = nxt[1]            
            time.sleep(0.05)
            
            if nxt == goal:
                path.append(nxt)
                return path
            else:
                if nxt not in path:
                    goal_dist = round(((nxt[0]-goal[0])**2+(nxt[1]-goal[1])**2)**0.5, 2)
                    if goal_dist in queue:
                        queue[goal_dist].append(((nxt), path+[nxt]))
                    else:
                        queue[goal_dist] = [((nxt), path+[nxt])]
                    vertexList[nxt].button_obj.configure(bg='white', relief=SUNKEN)
                    current = nxt
                    root.update()
        counter+=1



rows = 10
columns = 10
vertexList = {}
step = 0
start = None
goal = None



root = Tk()
root.title('Pathfinder')



frame1 = Frame(root, bg='black', bd=2)

for row in range(rows):
    for column in range(columns):
               
        vertexList[(row, column)] = ButtonVertex(master=frame1, position =(row, column))
        vertexList[(row, column)].button_obj.grid(row=row, column=column)
        
frame1.bind_all("<ButtonRelease-1>", mouse_up)
frame1.bind_all("<B1-Motion>", mouse_motion)
        
frame1.pack()



frame2 = Frame(root)

label = Label(frame2, text='Choose a starting node.')
label.pack(side=LEFT)
stepButton = Button(frame2, text = 'NEXT', width=10, height=2, command = nextStep)
stepButton.pack(side = RIGHT)

frame2.pack()



root.mainloop()




