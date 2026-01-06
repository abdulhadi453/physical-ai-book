/**
 * API service for communication with the chatbot backend.
 *
 * Provides functions to query the book and check backend health.
 */

import type { QueryResponse, ErrorResponse, HealthResponse } from '../components/Chatbot/types';

const API_BASE_URL = 'http://127.0.0.1:8000/api/v1';

/**
 * Query the book with a natural language question.
 *
 * @param question - The user's question about the book
 * @returns Promise resolving to the query response with answer and sources
 * @throws Error if the request fails or the backend is unreachable
 */
export async function queryBook(question: string): Promise<QueryResponse> {
  console.log(`Sending query to backend: ${question}`);
  try {
    // Add timeout for network requests (60 seconds for RAG agent)
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), 60000);

    const response = await fetch(`${API_BASE_URL}/query`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ question }),
      signal: controller.signal,
    });

    console.log('Backend response status:', response.status);
    clearTimeout(timeoutId);

    if (!response.ok) {
      const error: ErrorResponse = await response.json();
      console.error('Backend error:', error);
      throw new Error(error.message || 'Request failed');
    }

    const data = await response.json();
    console.log('Backend response data:', data);
    return data;
  } catch (error) {
    console.error('Chatbot API error:', error);
    // Handle timeout
    if (error instanceof Error && error.name === 'AbortError') {
      throw new Error('Request timed out. Please try again.');
    }
    // Handle network errors
    if (error instanceof Error) {
      throw error;
    }
    throw new Error('Connection failed. Check if backend is running.');
  }
}

/**
 * Check the health status of the backend service.
 *
 * @returns Promise resolving to the health status
 * @throws Error if the health check fails
 */
export async function checkHealth(): Promise<HealthResponse> {
  try {
    const response = await fetch(`${API_BASE_URL}/health`);
    if (!response.ok) {
      throw new Error('Health check failed');
    }
    return await response.json();
  } catch (error) {
    throw new Error('Backend is not accessible.');
  }
}
