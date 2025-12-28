import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin, urlparse
from typing import List, Dict, Optional, Set
import logging
import hashlib
from config import Config

class Crawler:
    """Web crawler for extracting content from book websites."""

    def __init__(self):
        self.session = requests.Session()
        self.session.headers.update({
            'User-Agent': 'Mozilla/5.0 (compatible; BookCrawler/1.0; +https://example.com/bot)'
        })
        self.visited_urls: Set[str] = set()
        self.all_urls: Set[str] = set()

    def is_valid_url(self, url: str) -> bool:
        """Check if URL is valid and belongs to the same domain as the base URL."""
        base_domain = urlparse(Config.BOOK_WEBSITE_URL).netloc
        parsed_url = urlparse(url)

        # Check if it's an HTTPS URL and belongs to the same domain
        return (
            parsed_url.scheme in ['http', 'https'] and
            parsed_url.netloc == base_domain
        )

    def get_page_content(self, url: str) -> Optional[Dict]:
        """Fetch and extract content from a single page."""
        try:
            logging.info(f"Fetching content from {url}")
            response = self.session.get(url, timeout=30)
            response.raise_for_status()

            soup = BeautifulSoup(response.content, 'html.parser')

            # Extract content while preserving structure
            title = soup.find('title')
            title_text = title.get_text().strip() if title else ""
            logging.debug(f"Extracted title for {url}: {title_text[:50]}...")

            # Remove script and style elements
            for script in soup(["script", "style"]):
                script.decompose()

            # Extract headings and content
            headings = []
            for heading in soup.find_all(['h1', 'h2', 'h3', 'h4', 'h5', 'h6']):
                headings.append(heading.get_text().strip())

            # Extract main content (try to find main content areas)
            main_content = soup.find('main') or soup.find('article') or soup.find('div', class_='main-content') or soup
            content_text = main_content.get_text(separator=' ', strip=True)

            # Calculate content hash for idempotency
            content_hash = hashlib.sha256(content_text.encode()).hexdigest()
            logging.debug(f"Content hash calculated for {url}: {content_hash[:16]}...")

            result = {
                'url': url,
                'title': title_text,
                'headings': headings,
                'content': content_text,
                'metadata': {
                    'content_hash': content_hash,
                    'source_url': url
                }
            }

            logging.info(f"Successfully extracted content from {url} ({len(content_text)} chars, {len(headings)} headings)")
            return result
        except requests.exceptions.Timeout:
            logging.error(f"Timeout error crawling {url}: Request took longer than 30 seconds")
            return None
        except requests.exceptions.ConnectionError:
            logging.error(f"Connection error crawling {url}: Unable to connect to the server")
            return None
        except requests.exceptions.HTTPError as e:
            logging.error(f"HTTP error crawling {url}: {e.response.status_code} - {e.response.reason}")
            return None
        except requests.exceptions.RequestException as e:
            logging.error(f"Network error crawling {url}: {str(e)}")
            return None
        except Exception as e:
            logging.error(f"Unexpected error crawling {url}: {str(e)}")
            return None

    def find_links(self, url: str) -> List[str]:
        """Find all links on a page that belong to the same domain."""
        try:
            logging.info(f"Finding links on {url}")
            response = self.session.get(url, timeout=30)
            response.raise_for_status()

            # Parse HTML content with error handling for malformed HTML
            try:
                soup = BeautifulSoup(response.content, 'html.parser')
            except Exception as parse_error:
                logging.error(f"Error parsing HTML for {url}: {str(parse_error)}")
                # Try with a more lenient parser
                soup = BeautifulSoup(response.content, 'html.parser', features="lxml")
                logging.info(f"Successfully parsed HTML for {url} using fallback parser")

            links = []

            for link in soup.find_all('a', href=True):
                try:
                    absolute_url = urljoin(url, link['href'])
                    if self.is_valid_url(absolute_url):
                        # Only include links that are likely to be book content
                        parsed = urlparse(absolute_url)
                        path = parsed.path.lower()

                        # Include HTML pages and exclude certain file types
                        if (path.endswith(('.html', '.htm', '/')) or not any(
                            ext in path for ext in ['.pdf', '.jpg', '.jpeg', '.png', '.gif', '.zip', '.exe', '.doc', '.docx']
                        )):
                            links.append(absolute_url)
                except Exception as link_error:
                    logging.warning(f"Error processing link on {url}: {str(link_error)}")
                    continue  # Skip problematic links and continue with others

            logging.info(f"Found {len(links)} valid links on {url}")
            return links
        except requests.exceptions.Timeout:
            logging.error(f"Timeout error finding links on {url}: Request took longer than 30 seconds")
            return []
        except requests.exceptions.ConnectionError:
            logging.error(f"Connection error finding links on {url}: Unable to connect to the server")
            return []
        except requests.exceptions.HTTPError as e:
            logging.error(f"HTTP error finding links on {url}: {e.response.status_code} - {e.response.reason}")
            return []
        except requests.exceptions.RequestException as e:
            logging.error(f"Network error finding links on {url}: {str(e)}")
            return []
        except Exception as e:
            logging.error(f"Unexpected error finding links on {url}: {str(e)}")
            return []

    def crawl_website(self, start_url: str, max_pages: int = 100) -> List[Dict]:
        """Crawl the website starting from the given URL."""
        logging.info(f"Starting crawl of website: {start_url}, max pages: {max_pages}")
        self.all_urls = set()
        self.visited_urls = set()

        # Start with the initial URL
        to_visit = [start_url]
        results = []

        while to_visit and len(self.visited_urls) < max_pages:
            current_url = to_visit.pop(0)

            if current_url in self.visited_urls:
                continue

            self.visited_urls.add(current_url)
            logging.info(f"Crawling: {current_url} ({len(self.visited_urls)}/{max_pages})")

            # Extract content from current page
            content_data = self.get_page_content(current_url)
            if content_data:
                results.append(content_data)
                logging.debug(f"Successfully processed {current_url}")

            # Find new links to crawl
            new_links = self.find_links(current_url)
            for link in new_links:
                if link not in self.visited_urls and link not in to_visit:
                    to_visit.append(link)

        logging.info(f"Crawling completed. Visited {len(self.visited_urls)} pages, processed {len(results)} successfully.")
        return results

def validate_website_url(url: str) -> tuple[bool, str]:
    """Validate that the target website is accessible before starting processing."""
    try:
        if not Config.validate_url(url):
            error_msg = f"Invalid URL format: {url}. Must be a valid HTTPS URL."
            logging.error(error_msg)
            return False, error_msg

        logging.info(f"Validating website accessibility: {url}")
        response = requests.head(url, timeout=10)
        if response.status_code >= 400:
            error_msg = f"Website returned error status: {response.status_code}"
            logging.error(error_msg)
            return False, error_msg

        logging.info(f"Website validation successful: {url}")
        return True, "Website is accessible"
    except requests.exceptions.Timeout:
        error_msg = f"Timeout validating website: {url}"
        logging.error(error_msg)
        return False, error_msg
    except requests.exceptions.ConnectionError:
        error_msg = f"Connection error validating website: {url}"
        logging.error(error_msg)
        return False, error_msg
    except Exception as e:
        error_msg = f"Unable to access website: {str(e)}"
        logging.error(error_msg)
        return False, error_msg