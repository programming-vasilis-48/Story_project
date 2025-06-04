#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test script for the LLM client.

This script tests the LLM client by generating a sample response.
"""

import os
import sys
from dotenv import load_dotenv

# Add the modules directory to the Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Import the LLM client
from modules.llm_api.llm_client import LLMClient

def main():
    """Test the LLM client."""
    # Load environment variables
    load_dotenv()

    print("Testing LLM client...")

    # Create an instance of the LLM client with OpenRouter
    try:
        client = LLMClient(provider="openrouter")
        print(f"LLM client initialized successfully with OpenRouter")
        print(f"Using provider: {client.provider}")
        print(f"Using model: {client.model}")

        # Generate a test response
        system_prompt = (
            "You are a storytelling robot participating in a collaborative storytelling exercise with a human. "
            "Your responses should be exactly ONE sentence that continues the story. "
            "Your response should express the emotion: happy. "
            "Do not use quotation marks or prefixes like 'Robot:'. Just provide the sentence."
        )

        user_prompt = (
            "Here is our story so far:\n"
            "User: Once upon a time, there was a small village nestled in a valley.\n\n"
            "Continue the story with ONE sentence expressing happy. "
            "Remember to maintain continuity with what has been said before."
        )

        print("\nGenerating response...")
        response = client.generate_text(
            prompt=user_prompt,
            system_prompt=system_prompt,
            max_tokens=100,
            temperature=0.7
        )

        print("\nGenerated response:")
        print(response)

    except Exception as e:
        print(f"Error: {str(e)}")
        return 1

    return 0

if __name__ == "__main__":
    sys.exit(main())
