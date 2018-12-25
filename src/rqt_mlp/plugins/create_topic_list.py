# import rospy, rostopic
import rosgraph
import calc_bw

# rospy.init_node('blurglesplort')

master = rosgraph.Master('topics_list')

topic_data_list4 = map(lambda l: l[0], master.getPublishedTopics(''))
topic_data_list4.sort()

with open("topics.txt","w") as f:
    for item in topic_data_list4:
        is_larger_than_1MB = calc_bw.rostopicmain(["rostopic", "bw", item])
        # print is_larger_than_1MB
        if not is_larger_than_1MB:
            f.write(item+"\n")
            print item
print "done!"

