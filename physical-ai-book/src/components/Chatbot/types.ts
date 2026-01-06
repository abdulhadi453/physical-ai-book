/**
 * TypeScript type definitions for the Chatbot component.
 *
 * These types mirror the backend Pydantic models to ensure type safety.
 */

export interface SourceCitation {
  chunk_id: string;
  section: string;
  page?: number;
  text_preview: string;
  similarity_score: number;
}

export interface ChatMessage {
  id: string;
  type: 'user' | 'assistant';
  content: string;
  sources?: SourceCitation[];
  timestamp: Date;
}

export interface QueryRequest {
  question: string;
  scope?: string;
  options?: {
    max_results?: number;
    temperature?: number;
  };
}

export interface QueryResponse {
  success: boolean;
  answer: string;
  sources: SourceCitation[];
  latency_ms: number;
}

export interface ErrorResponse {
  success: boolean;
  error: string;
  message: string;
  details?: object;
}

export interface HealthResponse {
  status: string;
  qdrant_connected: boolean;
  openai_connected: boolean;
  timestamp: string;
}
