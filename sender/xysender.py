import sys
from autobahn.asyncio.component import Component
from autobahn.asyncio.component import run


comp = Component(
    transports=u"ws:..port../ws",
    realm=u"...realm...",
)

@comp.on_join
async def joined(session, details):
    print("session ready")
    
    a = [1112914.451, 7085238.525]
    session.publish(u'<session name for rcToXy Conversation>', a)

    sys.exit(0)

if __name__ == "__main__":
    run([comp])