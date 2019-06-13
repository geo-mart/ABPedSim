import sys
from autobahn.asyncio.component import Component
from autobahn.asyncio.component import run


comp = Component(
    transports=u"ws:..port../ws",
    realm=u"..realm..",
)

@comp.on_join
async def joined(session, details):
    print("session ready")
    
    peds = 30;
    mapExtent = [565900.000, 5933766.750, 566173.405, 5933998.937]
    ty = 1
    
    session.publish(u'<session name for ABPedSim Start>', mapExtent, peds, ty)

    sys.exit(0)

if __name__ == "__main__":
    run([comp])