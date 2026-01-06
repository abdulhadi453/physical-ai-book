"""
Test script to verify backend configuration and basic functionality.
"""

import asyncio
import sys
from pathlib import Path

# Add backend directory to path
sys.path.insert(0, str(Path(__file__).parent))

from agent import (
    Config,
    create_qdrant_service,
    create_rag_agent,
    RetrievalError,
    AgentError,
    OPENAI_AGENTS_AVAILABLE
)


def test_config():
    """Test configuration loading."""
    print("=== Testing Configuration ===")
    print(f"OPENROUTER_API_KEY: {bool(Config.OPENROUTER_API_KEY)}")
    print(f"MODEL_NAME: {Config.MODEL_NAME}")
    print(f"COHERE_API_KEY: {bool(Config.COHERE_API_KEY)}")
    print(f"QDRANT_URL: {Config.QDRANT_URL}")
    print(f"QDRANT_COLLECTION_NAME: {Config.QDRANT_COLLECTION_NAME}")
    print(f"MAX_RETRIEVAL_CHUNKS: {Config.MAX_RETRIEVAL_CHUNKS}")
    print(f"SIMILARITY_THRESHOLD: {Config.SIMILARITY_THRESHOLD}")
    print()

    try:
        Config.validate()
        print("[OK] Configuration validation: PASSED")
    except ValueError as e:
        print(f"[FAIL] Configuration validation: FAILED - {e}")
        return False
    print()
    return True


def test_qdrant_connection():
    """Test Qdrant connection."""
    print("=== Testing Qdrant Connection ===")
    qdrant_service = create_qdrant_service()

    if qdrant_service.connect():
        print("[OK] Qdrant connection: CONNECTED")
        print(f"   Collection: {Config.QDRANT_COLLECTION_NAME}")
        print()
        return qdrant_service
    else:
        print("[FAIL] Qdrant connection: FAILED")
        print()
        return None


def test_agent_initialization():
    """Test agent initialization."""
    print("=== Testing Agent Initialization ===")

    if not OPENAI_AGENTS_AVAILABLE:
        print("[FAIL] OpenAI Agents SDK not available")
        print("   Run: pip install openai-agents")
        return None

    qdrant_service = create_qdrant_service()
    if not qdrant_service.connect():
        print("[FAIL] Cannot initialize agent - Qdrant not connected")
        return None

    rag_agent = create_rag_agent(qdrant_service)

    if rag_agent.initialize():
        print("[OK] Agent initialization: SUCCESS")
        print(f"   Model: {Config.MODEL_NAME}")
        print(f"   Base URL: https://openrouter.ai/api/v1")
        print(f"   Tools: retrieve_book_content")
        print()
        return rag_agent
    else:
        print("[FAIL] Agent initialization: FAILED")
        print()
        return None


async def test_retrieval(qdrant_service):
    """Test content retrieval."""
    print("=== Testing Content Retrieval ===")

    test_query = "What is physical AI?"

    try:
        # Simple test - just check if we can get embeddings
        from agent import encode_query, retrieve_context_chunks

        query_vector = encode_query(test_query)
        print("[OK] Query embedding: SUCCESS")
        print(f"   Query: {test_query}")
        print(f"   Embedding dimensions: {len(query_vector)}")

        chunks = retrieve_context_chunks(test_query, qdrant_service, limit=3)
        print(f"[OK] Retrieved {len(chunks)} chunks")

        if chunks:
            for i, chunk in enumerate(chunks):
                print(f"   Chunk {i+1} (Score: {chunk['score']:.4f}, Section: {chunk['metadata'].get('section', 'Unknown')}):")
                print(f"   Text: {chunk['text'][:150]}...")
                print("-" * 20)
        else:
            print("   ! No chunks found (may be normal if collection is empty)")
        print()
        return True

    except Exception as e:
        print(f"[FAIL] Retrieval test: FAILED - {e}")
        import traceback
        traceback.print_exc()
        print()
        return False


async def test_agent_query(rag_agent, qdrant_service):
    """Test agent with a simple query."""
    print("=== Testing Agent Query ===")

    test_question = "What is physical AI?"

    try:
        answer, has_content, has_sources = await rag_agent.generate_response(test_question)
        print("[OK] Agent query: SUCCESS")
        print(f"   Question: {test_question}")
        print(f"   Has retrieved content: {has_content}")
        print(f"   Has sources: {has_sources}")
        print(f"   Answer preview: {answer[:200]}...")
        print()
        return True

    except Exception as e:
        print(f"[FAIL] Agent query: FAILED - {e}")
        import traceback
        traceback.print_exc()
        print()
        return False


async def run_all_tests():
    """Run all backend tests."""
    print("\n" + "="*50)
    print("Backend Verification Test Suite")
    print("="*50 + "\n")

    # Test 1: Configuration
    if not test_config():
        return

    # Test 2: Qdrant connection
    qdrant_service = test_qdrant_connection()
    if qdrant_service is None:
        print("! Continuing without Qdrant connection...")
        print()

    # Test 3: Agent initialization
    rag_agent = test_agent_initialization()
    if rag_agent is None:
        print("! Cannot proceed without agent")
        return

    # Test 4: Retrieval
    if qdrant_service:
        await test_retrieval(qdrant_service)

    # Test 5: Agent query
    if rag_agent:
        await test_agent_query(rag_agent, qdrant_service)

    print("="*50)
    print("[OK] All available tests completed!")
    print("="*50 + "\n")


if __name__ == "__main__":
    asyncio.run(run_all_tests())
