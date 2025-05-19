#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test script for the LLM API on QTrobot.
This script tests if the OpenRouter API works and can generate responses.
"""

import os
import sys
import requests
import json
from dotenv import load_dotenv

def test_llm_api():
    """Test the LLM API connection and response generation."""
    print("Testing LLM API connection...")
    
    # Load environment variables from .env file
    load_dotenv()
    
    # Get API key and model from environment variables
    api_key = os.getenv("OPENROUTER_API_KEY")
    model = os.getenv("MODEL", "meta-llama/llama-3.3-8b-instruct:free")
    
    if not api_key:
        print("Error: OPENROUTER_API_KEY environment variable is not set.")
        print("Please create a .env file with your API key.")
        return False
    
    print(f"Using model: {model}")
    
    # Set API endpoint
    api_url = "https://openrouter.ai/api/v1/chat/completions"
    
    # Prepare headers
    headers = {
        "Authorization": f"Bearer {api_key}",
        "Content-Type": "application/json"
    }
    
    # Prepare messages
    system_prompt = "You are a helpful assistant for QTrobot."
    user_prompt = "Generate a short greeting for a child interacting with a robot."
    
    messages = [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": user_prompt}
    ]
    
    # Prepare request data
    data = {
        "model": model,
        "messages": messages,
        "max_tokens": 100,
        "temperature": 0.7
    }
    
    try:
        print("Sending request to OpenRouter API...")
        # Make API request
        response = requests.post(api_url, headers=headers, json=data)
        response.raise_for_status()  # Raise exception for HTTP errors
        
        # Parse response
        result = response.json()
        
        # Extract generated text
        if "choices" in result and len(result["choices"]) > 0:
            generated_text = result["choices"][0]["message"]["content"].strip()
            print("\nAPI Response:")
            print(f"Generated text: {generated_text}")
            return True
        else:
            print("Error: No response from LLM")
            print(f"API response: {result}")
            return False
    
    except requests.exceptions.RequestException as e:
        print(f"Error: Failed to communicate with LLM API: {str(e)}")
        return False
    except json.JSONDecodeError:
        print("Error: Invalid response from LLM API")
        return False
    except Exception as e:
        print(f"Error: {str(e)}")
        return False

if __name__ == "__main__":
    success = test_llm_api()
    if success:
        print("\nLLM API test successful! ✅")
    else:
        print("\nLLM API test failed! ❌")
