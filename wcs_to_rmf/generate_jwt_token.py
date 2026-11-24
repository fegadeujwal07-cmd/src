#!/usr/bin/env python3
"""
Generate JWT token for RMF API Server authentication

This script generates a JWT token that matches the API server's configuration:
- Secret: "rmfisawesome" (default)
- Audience: "rmf_api_server"
- Issuer: "stub"
- Username: "admin" (default builtin admin)
- Expiration: Never expires by default (expires in year 2099, effectively permanent)
"""

import jwt
import sys
from datetime import datetime, timedelta

# API Server JWT Configuration (from default_config.py)
JWT_SECRET = "rmfisawesome"
JWT_AUDIENCE = "rmf_api_server"
JWT_ISSUER = "stub"
USERNAME = "admin"  # Default builtin admin user

def generate_token(username: str = USERNAME, secret: str = JWT_SECRET, 
                   audience: str = JWT_AUDIENCE, issuer: str = JWT_ISSUER,
                   expires_in_hours: int = None) -> str:
    """
    Generate a JWT token for RMF API Server authentication
    
    Args:
        username: Username to use (default: "admin")
        secret: JWT secret key (default: "rmfisawesome")
        audience: JWT audience claim (default: "rmf_api_server")
        issuer: JWT issuer claim (default: "stub")
        expires_in_hours: Token expiration time in hours. If None, token never expires (default: None)
                          Set to a number (e.g., 24) for specific expiration time.
        
    Returns:
        str: JWT token string
    """
    # Create token payload
    now = datetime.utcnow()
    payload = {
        "preferred_username": username,  # Required by API server
        "aud": audience,                 # Audience claim
        "iss": issuer,                   # Issuer claim
        "iat": int(now.timestamp()),     # Issued at
    }
    
    # Add expiration only if specified, otherwise token never expires
    # For never-expiring tokens, we set expiration to year 2099 (effectively permanent)
    if expires_in_hours is not None:
        payload["exp"] = int((now + timedelta(hours=expires_in_hours)).timestamp())
    else:
        # Set expiration to year 2099 (effectively never expires)
        # Using datetime(2099, 12, 31) as a far-future date
        future_date = datetime(2099, 12, 31, 23, 59, 59)
        payload["exp"] = int(future_date.timestamp())
    
    # Generate token using HS256 algorithm
    token = jwt.encode(payload, secret, algorithm="HS256")
    
    return token

if __name__ == "__main__":
    # Parse command line arguments
    username = sys.argv[1] if len(sys.argv) > 1 else USERNAME
    secret = sys.argv[2] if len(sys.argv) > 2 else JWT_SECRET
    
    # Generate token
    token = generate_token(username=username, secret=secret)
    
    print("=" * 60)
    print("RMF API Server JWT Token")
    print("=" * 60)
    print(f"\nToken (copy this):")
    print(token)
    print(f"\nTo use this token, set the environment variable:")
    print(f"export RMF_API_JWT_TOKEN=\"{token}\"")
    print(f"\nOr add it to your server startup command:")
    print(f"RMF_API_JWT_TOKEN=\"{token}\" python3 -m uvicorn wcs_to_rmf.mission_api:app --host 0.0.0.0 --port 1234")
    print("=" * 60)




