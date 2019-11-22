import math
from scipy.integrate import dblquad
try:
	import Queue as Q
except ImportError:
	import queue as Q

size = 1662
Trans_mat_size = 1662
no_of_obs_point = 31
nearest_seg = 2
delta_t = 60.0
sigma_z = 5.0
sigma_v = 5.0
window_size = 4; # knobe : incresing window_size increase both accuracy and time complexity

Trans_matrix = [[0 for x in range(Trans_mat_size)] for y in range(Trans_mat_size)]
#initial = [1.0]*Trans_mat_size
save_prev_max1 = [0]*nearest_seg
route = [0]*no_of_obs_point
adjmat = [[0 for x in range(size)] for y in range(size)]
idmat = [[0 for x in range(size)] for y in range(size)]
prev_road = [0]*nearest_seg
min_trans = 0.0001   # knobe minimum the value of min_trans accurate will be the result but time complexity increase 
emission_queue1 = Q.Queue()
emission_queue2 = Q.Queue()

mapnode = {}

class lat_long_touple:
	def __init__(self):
		self.id = -1
		self.id1 = -1
		self.id2 = -1
		self.lt1 = -1.0
		self.lg1 = -1.0
		self.lt2 = -1.0
		self.lg2 = -1.0

class llpair:
	def __init__(self):
		self.lt = -1.0
		self.ig = -1.0

maproad = {} # save road's details id=road_id, id1=id_of_vertex, id2=id_of_another_vertex, lt1 and lt2 belongs to 1st vertex....
mapnode_llpair = {}


########################################### great circle distance ################################################################
def distance(lat1,long1,lat2,long2):
	one_deg = math.pi / 180
	# Convert the latitudes and longitudes from degree to radians. 
	lat1 = one_deg * lat1
	long1 = one_deg * long1 
	lat2 = one_deg * lat2
	long2 = one_deg * long2 
	dlong = long2 - long1 
	dlat = lat2 - lat1
	ans = math.pow(math.sin(dlat / 2), 2) + math.cos(lat1) * math.cos(lat2) * math.pow(math.sin(dlong / 2), 2) # Haversine Formula 
	ans = 2 * math.asin(math.sqrt(ans)) 
	R = 6371.0 # Radius of Earth in Kilometers, R = 6371  Use R = 3956 for miles 
	ans = ans * R 
	return ans

#####################################################################################################################################
#####################################################################################################################################
def distance_road_gps(lat1,long1,s1):
	slope = (s1.lg2 - s1.lg1)/(s1.lt2 - s1.lt1)
	result = llpair()
	result.lt = (long1 - s1.lg1 + (slope*s1.lt1) + (lat1/slope)) / (slope + (1/slope))
	result.lg = ((slope*result.lt) - (slope*s1.lt1) + s1.lg1)
	if( (s1.lt1>result.lt  and  s1.lt2>result.lt) or (s1.lt1<result.lt  and  s1.lt2<result.lt) or (s1.lg1>result.lg  and  s1.lg2>result.lg) or (s1.lg1<result.lg  and  s1.lg2<result.lg) ):
		d1 = distance(lat1,long1,s1.lt1,s1.lg1)
		d2 = distance(lat1,long1,s1.lt2,s1.lg2)
		if(d1>d2):
			result.lt = s1.lt2
			result.lg = s1.lg2
		elif(d2>=d1):
			result.lt = s1.lt1
			result.lg = s1.lg1
	dis = distance(lat1,long1,result.lt,result.lg)
	return (dis*1000) #//1000 to convert km to m

#####################################################################################################################################
################################################# nearest road segment  #############################################################
'''
this function return lat_long_touple having id1 = id of 1st nearest road
and id2 = id of 2nd nearest road
rest all field are garbage and of no use till now
'''
def find_2_near_seg(lat1,long1,id_prev): #//worst case O(NoOfRoads)
	res = lat_long_touple()
	min1=10000000.0
	min2=10000000.0
	for j in range(0,size):
		if(Trans_matrix[id_prev][j]>min_trans):
			lltpl = lat_long_touple()
			lltpl = maproad[j]
			dis = distance_road_gps(lat1,long1,lltpl)
			if(min1>=dis  and  min2>=dis):
				min2=min1
				min1=dis
				res.id2=res.id1
				res.id1=j
			elif(min1<dis  and  min2>=dis):
				min2=dis
				res.id2=j

	if(min1==min2 or min2==10000000):
		res.id2=res.id1

	return res

#####################################################################################################################################
#####################################################################################################################################
def calc_emission_prob(lat1,long1,id_prev,id_gps): #//worst case O(NoOfRoads)

	llt = find_2_near_seg(lat1,long1,id_prev) 
	llt1 = maproad[(llt.id1)]
	llt2 = maproad[(llt.id2)]
	llpair1 = return_projections(lat1,long1,llt.id1);
	llpair2 = return_projections(lat1,long1,llt.id2);

	great_circle_dis = distance_road_gps(lat1,long1,llt1)
	store = (1/(math.sqrt(2*math.pi)*sigma_z))*math.exp(-0.5*(great_circle_dis/sigma_z)*(great_circle_dis/sigma_z))
	if(llt.id1<llt.id2):
		emission_queue1.put( (llt.id1,llpair1,store) )
	else:
		emission_queue2.put( (llt.id1,llpair1,store) )

	great_circle_dis = distance_road_gps(lat1,long1,llt2);
	great_circle_dis = distance_road_gps(lat1,long1,llt2)
	store = (1/(math.sqrt(2*math.pi)*sigma_z))*math.exp(-0.5*(great_circle_dis/sigma_z)*(great_circle_dis/sigma_z))
	if(llt.id2<llt.id1):
		emission_queue1.put( (llt.id2,llpair2,store) )
	else:
		emission_queue2.put( (llt.id2,llpair2,store) )

#####################################################################################################################################
#####################################################################################################################################
def viterbi(start,curr_no_of_obs_point): #// O(window_size)
	save_prev_road = [0]*nearest_seg
	save_prev_max2 = [0]*nearest_seg
	save_idprev = start

	for curr_obs in range(start,curr_no_of_obs_point):
		save_prev_max2[0] = save_prev_max1[0]
		save_prev_max2[1] = save_prev_max1[1]
		z=0
		road_in_queue1,llpair1, b1 = emission_queue1.get()
		road_in_queue2,llpair2, b2 = emission_queue2.get()
		for curr_road in range(0,Trans_mat_size): # only 2 times
			b = 0.0
			if(curr_road == road_in_queue1):
				b = b1
			if(curr_road == road_in_queue2):
				b = b2

			if(b > 0.0):
				if(curr_obs==start):
					save_prev_max1[z] = 10000000.0 * 1.0 * b # here 1.0 should be replaced by initial[curr_road]
					prev_road[z]=curr_road
					save_prev_road[z]=curr_road
					z = z+1
				else:
					a = Trans_matrix[prev_road[0]][curr_road]
					maximum = -1.0
					for itr in range(0,nearest_seg):
						a = Trans_matrix[prev_road[itr]][curr_road]
						temp = save_prev_max2[itr] * a * b
						if(temp>=maximum):
							maximum = temp
							save_prev_max1[z] = maximum
					save_prev_road[z]=curr_road
					z=z+1

		maxim = -1 # by default
		for itr in range(0,nearest_seg):
			prev_road[itr] = save_prev_road[itr]
			if(save_prev_max1[itr] > maxim):
				maxim = save_prev_max1[itr]
				route[curr_obs] = prev_road[itr]
		
		print("R",route[curr_obs]," -> \t",end='')
		for itr in range(0,nearest_seg):
			print(save_prev_max1[itr],end="   ")

		if(route[curr_obs]==road_in_queue1):
			print("\t\t(",'{0:.6f}'.format(llpair1.lt)," , ",'{0:.6f}'.format(llpair1.lg),")",end=' ')
		elif(route[curr_obs]==road_in_queue2):
			print("\t\t(",'{0:.6f}'.format(llpair2.lt)," , ",'{0:.6f}'.format(llpair2.lg),")",end=' ')	
		print("\n")
		
		emission_queue1.put( (road_in_queue1,llpair1,b1) )
		emission_queue2.put( (road_in_queue2,llpair2,b2) )

		save_idprev = route[curr_obs]

	#print("actual : ",save_idprev)
	return save_idprev

#####################################################################################################################################
def print_matrix(a,row,col):
	for i in range(row):
		for j in range(col):
			print('{0:.3f}'.format(a[i][j]),end = "\t")
		print()

#####################################################################################################################################
############################################# building transition matrix ############################################################

def insert_starting():
	arr = [0]*Trans_mat_size
	for i in range(Trans_mat_size):
		lltouple = maproad[i]
		for j in range(Trans_mat_size):
			is_connected = adjmat[lltouple.id1][j]
			if(is_connected==1):
				arr[i]=arr[i]+1
				z = idmat[lltouple.id1][j]
				Trans_matrix[i][z] = 1.0
			is_connected = adjmat[lltouple.id2][j]
			if(is_connected==1):
				arr[i]=arr[i]+1
				z = idmat[lltouple.id2][j]
				Trans_matrix[i][z] = 1.0
		arr[i] = arr[i]-1;
	for i in range(Trans_mat_size):
		for j in range(Trans_mat_size):
			Trans_matrix[i][j]=(Trans_matrix[i][j] / arr[i])


def complete_trans_mat():
	for i in range(Trans_mat_size):
		for j in range(Trans_mat_size):
			if(Trans_matrix[i][j]>0 and i!=j):
				for k in range(Trans_mat_size):
					if(Trans_matrix[j][k]>0 and j!=k):
						if(Trans_matrix[i][k]==0):
							#print("b/w road ",i," and ",k," via ",j)
							Trans_matrix[i][k] = Trans_matrix[i][j] * Trans_matrix[j][k]
						else:
							Trans_matrix[i][k]= max((Trans_matrix[i][j] * Trans_matrix[j][k]),Trans_matrix[i][k])



#####################################################################################################################################
############################################### function to return projections ######################################################
'''
	input: latitude and longitude of gps points and road_id
	output: latitudes and longitudes of projections on road (lt,lg)
''' 
def return_projections( lat1, long1,idroad):
	# lat1 and long1 belongs to gps
	s1 = lat_long_touple() #lat and long of vertices of road
	s1 = maproad[idroad]
	slope = (s1.lg2 - s1.lg1)/(s1.lt2 - s1.lt1)
	result = llpair()
	result.lt = (long1 - s1.lg1 + (slope*s1.lt1) + (lat1/slope)) / (slope + (1/slope))
	result.lg = ((slope*result.lt) - (slope*s1.lt1) + s1.lg1)

	if( (s1.lt1>result.lt  and  s1.lt2>result.lt) or (s1.lt1<result.lt  and  s1.lt2<result.lt) or (s1.lg1>result.lg  and  s1.lg2>result.lg) or (s1.lg1<result.lg  and  s1.lg2<result.lg) ):
		d1 = distance(lat1,long1,s1.lt1,s1.lg1)
		d2 = distance(lat1,long1,s1.lt2,s1.lg2)
		if(d1>d2):
			result.lt = s1.lt2
			result.lg = s1.lg2
		else:
			result.lt = s1.lt1
			result.lg = s1.lg1

	return result

#####################################################################################################################################
#################################################### prediction #####################################################################
'''
formula: p(T) = product_over_i( p[e(i) -> e(i+1)] )
			p(r(k+1)|r(k),T) = int{ p_x * int{ p_v * p(r(k+1)|r(k),T,x,v) } }
'''
def road_length(id):
	lltpl = maproad[id]
	dist = distance(lltpl.lt1,lltpl.lg1,lltpl.lt2,lltpl.lg2)
	return dist * 1000  # 1000 for km to m

#####################################################################################################################################
def prediction(curr_road,en):
	vm = 12.0
	rk_l = road_length(curr_road)
	p_T = Trans_matrix[curr_road][en]
	T_1_to_n_l = road_length(en) + road_length(curr_road)
	T_1_to_n_minus_1_l = road_length(curr_road)
	def p_v(v):
		return (1/(math.sqrt(2*math.pi)*sigma_v))*math.exp(-0.5*((v-vm)/sigma_v)*((v-vm)/sigma_v))
	def p_x():
		return (1/rk_l)
	def ll(x):
		return (T_1_to_n_minus_1_l - x)/delta_t
	def ul(x):
		return (T_1_to_n_l - x)/delta_t

	area = dblquad(lambda v, x: (p_x())*(p_v(v)), 0, rk_l, lambda v: ll(v), lambda v: ul(v))
	#print(area[0])
	p_res = area[0] * p_T
	return p_res

def prediction_wrapper(curr_road):
	maximum = 0.0
	road_predicted = curr_road 
	for en in range(0,size):
		if(Trans_matrix[curr_road][en]>min_trans):
			save = prediction(curr_road,en)
			#print(en,save)
			if(save>=maximum):
				maximum = save
				road_predicted = en
	#print("road predicted : ",road_predicted)			
	return road_predicted

#####################################################################################################################################
#################################################### driver code ####################################################################
def main():
	
	fp = open("Node_data_sec43.txt","r")
	nm = ""
	lt1 = 0.0
	lg1 = 0.0
	for line in fp:
		nm, lt1, lg1 = [str(i) for i in line.split()]
		lt1 = float(lt1)
		lg1 = float(lg1)
		l = llpair()
		l.lt = lt1
		l.lg = lg1
		if(mapnode_llpair.get(nm)==None):
			mapnode_llpair[nm] = l

	fp.close()
	name1=""
	name2=""
	index=0
	fp = open("road_network_sec43.txt","r")
	for line in fp:
		name1, name2 = [str(i) for i in line.split()]
		if(mapnode.get(name1)==None):
			mapnode[name1] = index
			index = index +1
		if(mapnode.get(name2)==None):
			mapnode[name2] = index
			index = index +1
	fp.close()

	for i in range(0,size):
		for j in range(0,size):
			adjmat[i][j]=0
			idmat[i][j]=-1

	fp = open("road_network_sec43.txt","r")
	id_no = 0
	for line in fp:
		name1, name2 = [str(i) for i in line.split()]
		adjmat[mapnode[name1]][mapnode[name2]] = 1
		adjmat[mapnode[name2]][mapnode[name1]] = 1
		idmat[mapnode[name1]][mapnode[name2]] = id_no
		idmat[mapnode[name2]][mapnode[name1]] = id_no
		if(maproad.get(id_no)==None):
			lltouple = lat_long_touple()
			lltouple.id = id_no
			lltouple.lt1 = mapnode_llpair[name1].lt
			lltouple.lg1 = mapnode_llpair[name1].lg
			lltouple.lt2 = mapnode_llpair[name2].lt
			lltouple.lg2 = mapnode_llpair[name2].lg 
			lltouple.id1 = mapnode[name1]
			lltouple.id2 = mapnode[name2]
			maproad[id_no] = lltouple
		id_no = id_no + 1
	fp.close()

	insert_starting() # making trans_mat
	complete_trans_mat() # making trans_mat

	fp = open("gps_data_sec43.txt","r")
	lt2=0.0
	lg2=0.0
	line = fp.readline()
	lt1, lg1 = [str(i) for i in line.split()]
	lt1 = float(lt1)
	lg1 = float(lg1)
	calc_emission_prob(lt1,lg1,103,0) # mark
	id_pr = 103 # mark
	id_gps = 1
	curr_no_of_obs_point = 1
	start = 0
	#print_matrix(adjmat,Trans_mat_size,Trans_mat_size)
	#print_matrix(Trans_matrix,Trans_mat_size,Trans_mat_size)
	for line in fp:
		prediction_wrapper(id_pr)
		id_pr = viterbi(start,curr_no_of_obs_point)
		lt2, lg2 = [str(i) for i in line.split()]
		lt2 = float(lt2)
		lg2 = float(lg2)
		calc_emission_prob(lt2,lg2,id_pr,id_gps)
		lt1=lt2
		lg1=lg2
		curr_no_of_obs_point = curr_no_of_obs_point +1
		id_gps = id_gps + 1
		if(curr_no_of_obs_point>window_size):
			start = start + 1
			if(emission_queue1.empty() == False):
				emission_queue1.get()
			if(emission_queue2.empty() == False):
				emission_queue2.get()
		print("\n")
	fp.close()

	#print_matrix(Emission_matrix,no_of_obs_point,Trans_mat_size)

	prediction_wrapper(id_pr)
	viterbi(start,curr_no_of_obs_point)
	print("\n\nFinal route: ")
	for i in range(0,curr_no_of_obs_point-1):
		if(route[i+1]!=route[i]):
			print("R",route[i],end = " -> ")
	if(route[curr_no_of_obs_point-1]):
		print("R",route[curr_no_of_obs_point-1],"\n\n")
	
#####################################################################################################################################
	
if __name__ == "__main__":
	main()