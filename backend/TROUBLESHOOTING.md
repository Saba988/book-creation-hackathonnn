# Troubleshooting Guide: URL Ingestion and Embedding Pipeline

This guide provides solutions for common issues encountered when running the URL ingestion and embedding pipeline.

## Table of Contents
- [Common Issues](#common-issues)
- [Network and Connectivity Issues](#network-and-connectivity-issues)
- [API and Authentication Issues](#api-and-authentication-issues)
- [Content Processing Issues](#content-processing-issues)
- [Embedding Generation Issues](#embedding-generation-issues)
- [Storage Issues](#storage-issues)
- [Configuration Issues](#configuration-issues)
- [Performance Issues](#performance-issues)
- [Debugging Tips](#debugging-tips)

## Common Issues

### Pipeline Fails to Start
**Symptoms**: Pipeline exits immediately with an error message

**Possible Causes**:
1. Missing or invalid configuration
2. Missing Python dependencies
3. Invalid command line arguments

**Solutions**:
1. Check that all required environment variables are set in `.env`:
   ```bash
   cat .env
   ```
2. Verify dependencies are installed:
   ```bash
   pip list | grep -E "(cohere|qdrant|beautifulsoup4|requests)"
   ```
3. Check command line arguments:
   ```bash
   python main.py --help
   ```

### No URLs Processed Successfully
**Symptoms**: All URLs show as failed in the output

**Possible Causes**:
1. URLs are not publicly accessible
2. Network connectivity issues
3. Invalid URL format

**Solutions**:
1. Verify URLs are accessible in a browser
2. Check network connectivity: `curl -I <URL>`
3. Validate URL format follows `https://` or `http://` scheme

## Network and Connectivity Issues

### URL Ingestion Timeouts
**Symptoms**: URLs fail with timeout errors

**Solutions**:
1. Increase timeout in the ingestion module by modifying the timeout parameter
2. Check network connectivity to the target URLs
3. Verify the target servers are not rate-limiting your requests

### Connection Errors
**Symptoms**: `ConnectionError` or `ConnectionRefusedError` messages

**Solutions**:
1. Check firewall settings
2. Verify proxy settings if applicable
3. Ensure target URLs are accessible from your network

### Rate Limiting
**Symptoms**: 429 status codes or API limit exceeded errors

**Solutions**:
1. Add delays between requests in the ingestion module
2. Check your Cohere and Qdrant usage limits
3. Consider processing fewer URLs simultaneously

## API and Authentication Issues

### Invalid API Key
**Symptoms**: Authentication errors from Cohere or Qdrant

**Solutions**:
1. Verify API keys in `.env` file:
   - Ensure no extra spaces or quotes
   - Confirm keys are valid and active
   - Check for typos in the keys
2. Regenerate API keys if they've been compromised or expired

### Cohere API Errors
**Symptoms**: Embedding generation fails with Cohere-specific errors

**Solutions**:
1. Check your Cohere account limits
2. Verify the model name is valid
3. Ensure the content being embedded doesn't exceed size limits

### Qdrant Connection Issues
**Symptoms**: Storage operations fail with connection errors

**Solutions**:
1. Verify Qdrant URL is correct and accessible
2. Check that the API key is valid
3. Ensure the Qdrant Cloud instance is running and not in a paused state

## Content Processing Issues

### Malformed HTML
**Symptoms**: Content extraction fails or produces unexpected results

**Solutions**:
1. The pipeline uses BeautifulSoup for parsing; if specific sites consistently fail, they may have unusual HTML structures
2. Check logs for specific error messages
3. Consider pre-processing problematic URLs separately

### Empty Content Extraction
**Symptoms**: URLs are processed successfully but return empty content

**Solutions**:
1. Some sites load content dynamically with JavaScript (not supported by this pipeline)
2. Verify the site doesn't require authentication
3. Check if the site blocks automated requests

### Encoding Issues
**Symptoms**: Content contains strange characters or encoding errors

**Solutions**:
1. The pipeline handles common encodings automatically
2. If specific encoding issues occur, the cleaning module can be extended

## Embedding Generation Issues

### Embedding Generation Failure
**Symptoms**: Content chunks fail to generate embeddings

**Solutions**:
1. Check if content exceeds Cohere's size limits
2. Verify Cohere API key and model name
3. Ensure content is not empty or malformed

### High API Costs
**Symptoms**: Unexpectedly high Cohere API usage

**Solutions**:
1. Review chunk size settings - smaller chunks create more embeddings
2. Consider using different models with different pricing
3. Implement rate limiting to avoid unexpected usage spikes

## Storage Issues

### Qdrant Storage Failures
**Symptoms**: Successfully generated embeddings fail to store in Qdrant

**Solutions**:
1. Check Qdrant Cloud storage limits
2. Verify collection name and schema are correct
3. Ensure network connectivity to Qdrant Cloud

### Collection Creation Issues
**Symptoms**: Pipeline fails during collection creation phase

**Solutions**:
1. Verify Qdrant API key has collection creation permissions
2. Check if collection name is valid (no special characters)
3. Ensure Qdrant Cloud instance has sufficient resources

## Configuration Issues

### Environment Variables Not Loaded
**Symptoms**: Pipeline fails with missing API key errors despite `.env` file

**Solutions**:
1. Ensure `.env` file is in the correct directory
2. Verify the `.env` file is properly formatted
3. Check that python-dotenv is installed: `pip install python-dotenv`

### Invalid Configuration Values
**Symptoms**: Pipeline starts but fails with validation errors

**Solutions**:
1. Check all configuration values in `.env`:
   - URLs should start with `http://` or `https://`
   - API keys should not contain spaces or special characters
   - Numeric values should be valid numbers

## Performance Issues

### Slow Processing
**Symptoms**: Pipeline takes longer than expected to complete

**Solutions**:
1. Check network connectivity to target URLs
2. Verify API response times from Cohere
3. Consider increasing chunk size to reduce the number of API calls
4. Process fewer URLs simultaneously to avoid rate limits

### High Memory Usage
**Symptoms**: Pipeline fails with memory errors on large inputs

**Solutions**:
1. Reduce the number of URLs processed in a single run
2. Increase chunk size to reduce the number of chunks
3. Monitor system memory usage during execution

### API Rate Limits
**Symptoms**: Pipeline fails partway through with rate limit errors

**Solutions**:
1. Add delays between API calls
2. Reduce concurrent processing
3. Check your API provider's rate limit documentation

## Debugging Tips

### Enable Debug Logging
Run the pipeline with debug logging to get detailed information:
```bash
python main.py --urls "https://example.com" --log-level DEBUG
```

### Check Log Files
The pipeline creates `pipeline.log` with detailed execution information:
```bash
tail -f pipeline.log
```

### Test Individual Components
Test specific components separately:
```bash
# Test URL ingestion
python -c "from ingestion import ingest_urls; print(ingest_urls(['https://example.com']))"

# Test content cleaning
python -c "from cleaning import clean_content; print(clean_content('  messy   content  '))"
```

### Validate Configuration
Before running the full pipeline, validate your configuration:
```bash
python -c "from config import Config; Config.validate(); print('Configuration is valid')"
```

### Monitor API Usage
Check your Cohere and Qdrant usage dashboards regularly to avoid unexpected charges or limits.

### Common Validation Steps
1. Verify all required environment variables are set
2. Test URL accessibility: `curl -I https://example.com`
3. Test API keys independently of the pipeline
4. Start with a small number of test URLs before processing large batches

## Getting Help

If you encounter issues not covered in this guide:

1. Check the detailed logs in `pipeline.log`
2. Verify all dependencies are up-to-date
3. Review the README for any missed setup steps
4. Consider running with `--log-level DEBUG` for more detailed information