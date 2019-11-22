import re
#import requests
def main():
	#////////////////////////////////////////////////api access/////////////////////////
	# Make a get request to get the latest position of the international space station from the opennotify api.
	#data = requests.get("https://api.openstreetmap.org/api/0.6/map?bbox=76.7467,30.7169,76.7551,30.7245")
	# Print the status code of the response.
	#print(data.content)
	#with open('feed.osm', 'wb') as f: 
		#f.write(data.content) 




	#///////////////////////////////////////////////////////////////////////////////////

	fp = open("map.osm","r")
	file1 = open("Node_data_sec43.txt","w") 
	file2 = open("road_network_sec43.txt","w") 
	pattern1 = 'id="[0-9]*"'
	pattern2 = 'lat="[0-9]*.[0-9]*"'
	pattern3 = 'lon="[0-9]*.[0-9]*"'
	pattern4 = 'ref="[0-9]*"'
	pattern5 = '<nd'
	pattern6 = '</way>'
	j =0
	k =0
	for line in fp:
		print("line :",k+1)
		k=k+1
		match1 = re.search(pattern1,line)
		match2 = re.search(pattern2,line)
		match3 = re.search(pattern3,line)
		match4 = re.search(pattern4,line)
		match5 = re.search(pattern5,line)
		match6 = re.search(pattern6,line)
		if match1:
			if match2:
				if match3:
					id = match1.group()
					lat = match2.group()
					lon = match3.group()
					numbers = re.findall('\d+',id) 
					id = int(numbers[0])
					numbers = re.findall('\d+\.\d+',lat)
					lat = float(numbers[0])
					numbers = re.findall('\d+\.\d+',lon)
					lon = float(numbers[0])
					print("id:",id," lat:",lat," lon:",lon)
					file1.write("%i\t%.7f\t%.7f\n" % (id, lat ,lon))
					j = 0
		elif match5 and match4:
			ref = match4.group()
			numbers = re.findall('\d+',ref) 
			ref = int(numbers[0])
			if(j==0):
				node1 = ref
				j = j+1
			else:
				node2 = ref
				file2.write("%i\t%i\n" % (node1, node2))
				print("node1:",node1," node2:",node2)
				node1 = node2
		elif match6:
			j =0
	fp.close()
	file1.close()
	file2.close()




	
if __name__ == "__main__":
	main()
