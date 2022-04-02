#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <algorithm>
#include <climits>
#include <iomanip>
#include "rapidxml.hpp"
#define ll long long
using namespace std;
using namespace rapidxml;

//base class(parent to way and node classes)
class element {
    public:
        //attributes
        ll id;
        bool visible = 1;
        int version = 1;
        ll changeset;
        string timestamp;
        string user;
        ll uid;
        unordered_map<string, string> tag;
};

//node class
class node : public element {
    public:
        //attributes
        double lat;
        double lon;

        //methods
        void initnode(xml_node<> *node) {       //initialise node
            id = atoll(node->first_attribute("id")->value());
            if((node->first_attribute("visible")->value())[0]!='t') visible = 0;
            else visible = 1;
            version = atoi(node->first_attribute("version")->value());
            changeset = atoll(node->first_attribute("changeset")->value());
            timestamp = node->first_attribute("timestamp")->value();
            user = node->first_attribute("user")->value();
            uid = atoll(node->first_attribute("uid")->value());
            lat = stod(node->first_attribute("lat")->value());
            lon = stod(node->first_attribute("lon")->value());
            tag.clear();
            //parse tags attached to this node
            for(xml_node<> * temp = node->first_node("tag"); temp; temp = temp->next_sibling("tag")) {
                tag.emplace(make_pair(temp->first_attribute("k")->value(), temp->first_attribute("v")->value()));
            }
        }
        void display() {                        //display node
            cout<<"\n\t\tID: "<<id
                <<"\n\t\tVisibility: "<<(visible? "true": "false")
                <<"\n\t\tVersion: "<<version
                <<"\n\t\tChangeset: "<<changeset
                <<"\n\t\tTimestamp: "<<timestamp
                <<"\n\t\tUser ID: "<<uid
                <<"\n\t\tUsername: "<<user
                <<"\n\t\tLatitude: "<<setprecision(3)<<lat
                <<"\n\t\tLongitude: "<<setprecision(3)<<lon;
            if(!tag.empty()) {
                unordered_map<string, string>:: iterator itr;
                cout << "\n\t\tTags : ";
                for (itr = tag.begin(); itr != tag.end(); itr++) {
                    cout <<"\n\t\t\t"<< itr->first <<" :\t\t"<< itr->second;
                }
            }
            cout<<'\n';
            return;
        }
};

class way : public element {
    public:
        //attribute
        vector<ll> nd;

        //methods
        void initway(xml_node<> * way) {        //initialise way
            ll t;
            id = atoll(way->first_attribute("id")->value());
            if((way->first_attribute("visible")->value())[0]!='t') visible = 0;
            else visible = 1;
            version = atoi(way->first_attribute("version")->value());
            changeset = atoll(way->first_attribute("changeset")->value());
            timestamp = way->first_attribute("timestamp")->value();
            user = way->first_attribute("user")->value();
            uid = atoll(way->first_attribute("uid")->value());
            tag.clear();
            nd.clear();
            //parse nodes contained in this way
            for(xml_node<> * temp = way->first_node("nd"); temp; temp = temp->next_sibling("nd")) {
                t = atoll(temp->first_attribute("ref")->value());
                nd.push_back(t);
            }
            //parse tags attached to this way
            for(xml_node<> * temp = way->first_node("tag"); temp; temp = temp->next_sibling("tag")) {
                tag.emplace(make_pair(temp->first_attribute("k")->value(), temp->first_attribute("v")->value()));
            }
        }
        void display() {                        //display way
            cout<<"\n\t\tID: "<<id
                <<"\n\t\tVisibility: "<<(visible? "true": "false")
                <<"\n\t\tVersion: "<<version
                <<"\n\t\tChangeset: "<<changeset
                <<"\n\t\tTimestamp: "<<timestamp
                <<"\n\t\tUser ID: "<<uid
                <<"\n\t\tUsername: "<<user
                <<"\n\t\tNumber of nodes: "<<nd.size();
            if(!tag.empty()) {
                unordered_map<string, string>:: iterator itr;
                cout << "\n\t\tTags : ";
                for (itr = tag.begin(); itr != tag.end(); itr++) {
                    cout <<"\n\t\t\t"<< itr->first <<" :\t\t"<< itr->second;
                }
            }
            cout<<'\n';
            return;
        }
};

//utility function to calculate approx. distance between two points on earth whose latitude and longitude is given
static double haversine(node &n1, node &n2) {
    //distance between latitudes and longitudes
    double dLat = (n2.lat - n1.lat) * M_PI / 180.0;
    double dLon = (n2.lon - n1.lon) * M_PI / 180.0;

    //convert to radians
    double l1 = (n1.lat) * M_PI / 180.0;
    double l2 = (n2.lat) * M_PI / 180.0;

    //apply formulae
    double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(l1) * cos(l2);
    double rad = 6378.1;
    double c = 2 * asin(sqrt(a));
    return rad * c;
}

//helper function for case insensitive substring matching
void convertlower(string &st) {
    for(int i=0; i<st.size(); i++) {
        st[i] = tolower(st[i]);
    }
    return;
}

//function to search for node(s) and way(s) by name
void searchbyname(unordered_map<ll, node> &nodelist, unordered_map<ll, way> &waylist) {

    //user input
    string st, st1;
    getchar();
    cout<<"\n\tEnter string to be searched: ";
    getline(cin, st);
    convertlower(st);
    
    //lists to store result
    vector<node> node_result;
    vector<way> way_result;
    
    //iterating over nodelist and waylist to find matches
    for(auto &n : nodelist) {
        if(!n.second.tag.empty()) {
            if(n.second.tag.find("name") != n.second.tag.end()) {
                st1 = n.second.tag["name"];
                convertlower(st1);
                if(st1.find(st) != string::npos) {
                    node_result.push_back(n.second);
                }
            }
        }
    }
    for(auto &w : waylist) {
        if(!w.second.tag.empty()) {
            if(w.second.tag.find("name") != w.second.tag.end()) {
                st1 = w.second.tag["name"];
                convertlower(st1);
                if(st1.find(st) != string::npos) {
                    way_result.push_back(w.second);
                }
            }
        }
    }
    //if no matches found
    if(node_result.empty() && way_result.empty()) {
        cout<<"\n\tNo matches found!";
        return;
    }
    //showing results
    cout<<"\n\t"<<node_result.size()<<" matching node(s) and "<<way_result.size()<<" matching way(s) found: ";
    if(!node_result.empty()) {
        cout<<"\n\n\t\tNode ID\t\t\t\tName\n";
        for(auto &n: node_result) cout<<"\n\t\t"<<n.id<<"\t\t\t"<<n.tag["name"];
    }
    if(!way_result.empty()) {
        cout<<"\n\n\t\tWay ID\t\t\t\tName\n";
        for(auto &w: way_result) cout<<"\n\t\t"<<w.id<<"\t\t\t"<<w.tag["name"];
    }
    
    char ans;
    cout<<"\n\n\tWould you like to see detailed results? Enter (y/Y) for affirmation, any other key to exit: ";
    cin>>ans;
    if(ans!='y' && ans!='Y') return;

    //displaying details of results
    cout<<"\n\n\t\tNumber of matching nodes found: "<<node_result.size()<<'\n';
    for(int i=0; i<node_result.size(); i++) node_result[i].display();

    cout<<"\n\n\t\tNumber of matching ways found: "<<way_result.size()<<'\n';
    for(int i=0; i<way_result.size(); i++) way_result[i].display();
}

//function to find  k nodes from source node src
void nearestnodes(ll src, unordered_map<ll, node> &nodelist, int k) {

    //data structure to store nodes with their distance from source in a sorted manner
    map<double, ll> dir;
    int i = 0;

    //populate with distances
    for(auto &n : nodelist) {
        dir.emplace(make_pair(haversine(n.second, nodelist[src]), n.second.id));
    }

    //Displaying the output
    cout<<"\n\tThe first "<<k<<" node(s) nearest to entered origin node(crow-fly distance) are:\n";
    map<double , ll> ::iterator itr = dir.begin();
    for(i=1; i<=k; i++) {
        itr++;
        cout<<"\n\t\tNode ID: "<<itr->second<<"\t\tDistance from origin: "<<setprecision(3)<<itr->first<<" kms";
    }
    return;
}

//utility function to populate the graph for all nodes in adjacency list form
void populategraph(unordered_map<ll, unordered_map<ll, double>>&graph, unordered_map<ll, node> &nodelist, unordered_map<ll, way> &waylist) {
    int i;
    double d;

    //iterate through all ways and emplace adjacent nodes and corresponding distance for each node
    for(auto &w: waylist) {
        if(graph[w.second.nd[0]].find(w.second.nd[1]) == graph[w.second.nd[0]].end()) {         //duplicate handling
            d = haversine(nodelist[w.second.nd[0]], nodelist[w.second.nd[1]]);
            graph[w.second.nd[0]].emplace(make_pair(w.second.nd[1], d));
            graph[w.second.nd[1]].emplace(make_pair(w.second.nd[0], d));
        }
        for(i=1; i<w.second.nd.size()-1; i++) {
            if(graph[w.second.nd[i]].find(w.second.nd[i+1]) == graph[w.second.nd[i]].end()) {   //duplicate handling
                d = haversine(nodelist[w.second.nd[i]], nodelist[w.second.nd[i+1]]);
                graph[w.second.nd[i]].emplace(make_pair(w.second.nd[i+1], d));
                graph[w.second.nd[i+1]].emplace(make_pair(w.second.nd[i], d));
            }
        }
    }

    return;
}

//auxilary function to display the shortest path found
void displaypath(ll id2, unordered_map<ll, pair<double, ll>> &dist) {
    if(dist[id2].second) displaypath(dist[id2].second, dist);
    cout<<"-> "<<id2<<" ( "<<setprecision(3)<<dist[id2].first<<" kms ) ";
}

//Shortest path algorithm to be applied for finding the shortest path
bool dijkstra(ll id1, ll id2, unordered_map<ll, unordered_map<ll, double>>&graph, unordered_map<ll, node>&nodelist, unordered_map<ll, way>&waylist) {
    //data structure to maintain the distance of all nodes from source
    unordered_map<ll, pair<double, ll>> dist;
    for(auto &g : graph) {
        dist.emplace(make_pair(g.first, make_pair(INT_MAX, 0)));
    }

    //data structure to add mark nodes whose shortest path is found
    unordered_set<ll> chosen;
    dist[id1] = make_pair(0, 0);
    ll id;
    double min;

    //iterative algorithm to find minimum distance node and update accordingly
    for(int ctr = 0; ctr < dist.size(); ctr++) {
        min=INT_MAX;
        id = 0;

        //finding node with minimum distance from source
        for(auto &d : dist) {
            if(chosen.find(d.first) == chosen.end() && d.second.first < min) {
                id = d.first;
                min = d.second.first;
            }
        }

        //exit loop if destination node is reached
        if(id == id2) {
            break;
        }

        //mark as reached
        if(id) chosen.emplace(id);
        else {  //If no path is available
            return 0;
        }

        //updating distance of adjacent nodes from selected node
        for(auto &n : graph[id]) {
            if(chosen.find(n.first) == chosen.end() && dist[n.first].first > dist[id].first + n.second) {
                dist[n.first].first = dist[id].first + n.second;
                dist[n.first].second = id;
            }
        }
    }

    //display the shortest path found
    cout<<"\n\tShortest path between given nodes found with distance "<<setprecision(3)<<dist[id2].first<<" kms."
        <<"\n\tWould you like to view this path? Enter (y/Y) for affirmaion, any other key to exit: ";
    char ans;
    cin>>ans;
    if(ans=='y' || ans=='Y') {
        cout<<"\nThe shortest path is:\n\nSTART ";
        displaypath(id2, dist);
        cout<<"-> END\n";
    }
    return 1;
}

void shiftnode(unsigned ll &id, unordered_map<ll, unordered_map<ll, double>> &graph, unordered_map<ll, node> &nodelist) {
    map<double, ll> near;
    for(auto &n : graph) {
        near.emplace(make_pair(haversine(nodelist[n.first], nodelist[id]), n.first));
    }
    cout<<"\tArgument node shifted from "<<id<<" to "<<near.begin()->second<<'\n';
    id = near.begin()->second;
}

//function called once in beginning of main to parse given input "map.osm" file
void parse_osm(unordered_map<ll, node> &nodelist, unordered_map<ll, way> &waylist, unordered_map<ll, unordered_map<ll, double>> &graph) {
    cout<<"\nParsing map data (map.osm) . . ." <<endl;

    xml_document<> doc;
    xml_node<> * root_node = NULL;

    // Read the map.osm file
    ifstream ifile ("map.osm");
    vector<char> buffer((istreambuf_iterator<char>(ifile)), istreambuf_iterator<char>());
    buffer.push_back('\0');

    // Parse the buffer
    doc.parse<0>(&buffer[0]);

    // Find out the root element
    root_node = doc.first_node("osm");

    node temp;
    // Iterate over the nodes and add them to nodelist
    for (xml_node<> * node = root_node->first_node("node"); node; node = node->next_sibling("node"))
    {   temp.initnode(node);
        nodelist.emplace(make_pair(temp.id, temp));
    }

    way temp1;
    // Iterate over the ways and add them to waylist
    for(xml_node<> *way = root_node->first_node("way"); way; way = way->next_sibling("way")) {
        temp1.initway(way);
        waylist.emplace(make_pair(temp1.id, temp1));
    }

    //prepare adjacency list graph for traversing nodes 
    populategraph(graph, nodelist, waylist);

    cout<<"\n\nInput file successfully parsed.\n\n";
    ifile.close();
    return;
}

   
int main() {

    //declare hash based containers for storing nodes and ways organised by their IDs
    unordered_map<ll, node> nodelist;
    unordered_map<ll, way> waylist;
    unordered_map<ll, unordered_map<ll, double>> graph;

    //call function for parsing map.osm file
    parse_osm(nodelist, waylist, graph);

    //Menu for user interface:
    int ch = 0;
    while(ch != 5) {
        cout<<"\n\n***********************************MENU***********************************\n\n"
            <<"1) Show file statistics\n"
            <<"2) Search -by matching substring\n"
            <<"3) Find nearest nodes (crow-fly distance)\n"
            <<"4) Calculate shortest path between nodes\n"
            <<"5) Exit\n"
            <<"\nEnter your choice: ";
        cin>>ch;
        while(ch<0 || ch>5) {
            cout<<"Enter valid choice(1-5): ";
            cin>>ch;
        }
        switch(ch) {
            case 1:
                cout<<"\n\tNumber of nodes found:\t\t\t\t"<<nodelist.size()
                    <<"\n\tNumber of ways found:\t\t\t\t"<<waylist.size()
                    <<"\n\tNumber of inaccesible nodes (not on a way):\t"<<nodelist.size()-graph.size()
                    <<"\n\n\tPress any key to return to menu.";
                getchar();
                getchar();
                break;
            
            case 2:
                searchbyname(nodelist, waylist);
                cout<<"\n\n\tPress any key to return to menu.";
                getchar();
                getchar();
                break;
            
            case 3:
                unsigned ll i, k;
                cout<<"\n\tEnter ID of the node: ";
                cin>>i;
                while(nodelist.find(i) == nodelist.end()) {
                    cout<<"\tNode with entered ID not found in file, enter valid ID: ";
                    cin>>i;
                }
                cout<<"\tEnter the number of nearest nodes to find: ";
                cin>>k;
                while(k>=nodelist.size()) {
                    cout<<"\tEntered value exceeds the maximum number of neighbour nodes found ("<<nodelist.size()-1<<"). Please enter a valid number: ";
                    cin>>k;
                }
                nearestnodes(i, nodelist, k);
                cout<<"\n\n\tPress any key to return to menu.";
                getchar();
                getchar();
                break;
            
            case 4:
                unsigned ll id1, id2;
                cout<<"\n\tEnter ID of starting node: ";
                cin>>id1;

                //handling erronous input
                while(nodelist.find(id1) == nodelist.end()) {
                    cout<<"\tNode with ID not found in file, enter valid ID: ";
                    cin>>id1;
                }
                cout<<"\tEnter ID of destination node: ";
                cin>>id2;
                while(nodelist.find(id2) == nodelist.end()) {
                    cout<<"\tNode with ID not found in file, enter valid ID: ";
                    cin>>id2;
                }
                char ans;

                //handling cases where node doesn't lie on any way
                if(graph.find(id1) == graph.end()) {
                    cout<<"\n\tSource node doesn't lie on any way, would you like to consider shifting it to nearest way node?"
                        <<"\n\tEnter [y,Y] to affirm, any other character to discard: ";
                    cin>>ans;
                    if(ans!='y' && ans!='Y') break;
                    shiftnode(id1, graph, nodelist);
                }
                if(graph.find(id2) == graph.end()) {
                    cout<<"\n\tDestination node doesn't lie on any way, would you like to consider shifting it to nearest way node?"
                        <<"\n\tEnter [y,Y] to affirm, any other character to discard: ";
                    cin>>ans;
                    if(ans!='y' && ans!='Y') break;
                    shiftnode(id2, graph, nodelist);
                }
                if(!dijkstra(id1, id2, graph, nodelist, waylist)) {
                    cout<<"\n\tSorry these nodes are disconnected, no path found!";
                };
                cout<<"\n\n\tPress any key to return to menu.";
                getchar();
                getchar();
                break;
        }
    }

    return 0;
}