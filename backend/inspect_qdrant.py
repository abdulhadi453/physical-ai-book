import os
from qdrant_client import QdrantClient
from dotenv import load_dotenv

load_dotenv()

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

collection_name = os.getenv("QDRANT_COLLECTION_NAME")
print(f"Inspecting collection: {collection_name}")

# Scroll to see the first 5 markers
points, _ = client.scroll(
    collection_name=collection_name,
    limit=5,
    with_payload=True,
    with_vectors=False
)

import sys
sys.stdout.reconfigure(encoding='utf-8')

for point in points:
    print(f"ID: {point.id}")
    print(f"Keys: {list(point.payload.keys())}")
    for key, value in point.payload.items():
        val_str = str(value)
        print(f"  {key}: {val_str[:200]}...")
    print("-" * 40)
