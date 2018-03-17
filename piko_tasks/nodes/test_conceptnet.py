#!/usr/bin/env python

import requests

from conceptnet5 import uri

db_uri = 'http://conceptnet5.media.mit.edu/data/5.2/'

# #uri_1 = db_uri + 'search?rel=/r/PartOf&end=/c/en/house&limit=10'
# #uri_2 = db_uri + 'search?rel=/r/IsA&end=/c/en/room&limit=10'
# 
# uri_3 = uri.conjunction_uri(['/r/PartOf', '/c/en/house'], ['/r/IsA', '/c/en/room'])
# 
# print uri_3

response = requests.get(db_uri + 'search?/and/&rel=/r/PartOf&end=/c/en/house&rel=/r/IsA&end=/c/en/room&limit=10')
#response = requests.get(db_uri + 'search?rel=/and[/r/PartOf&end=/c/en/house,/r/IsA&end=/c/en/room]&limit=10')

#response = requests.get(uri_3)

data = response.json()

for edge in data['edges']:
    part = edge['start']
    print part
#     response2 = requests.get(db_uri + 'search?rel=/r/PartOf&end=' + part + '&limit=10')
#     data2 = response2.json()
#     for edge2 in data2['edges']:
#         subpart = edge2['start']
#         print '   ', subpart
